#!/usr/bin/env python3
"""No-APP recovery tooling: plan locally, inspect/watch remotely, opt-in trial."""
import argparse
import base64
import json
import math
import os
from pathlib import Path
import select
import shlex
import subprocess
import sys
import time

from recovery_core import PROFILE, TARGET, packets, plan
from recovery_config import load_config


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode', choices=['plan', 'inspect', 'watch', 'manual', 'auto'])
    p.add_argument('--direction', choices=['left', 'right'])
    p.add_argument('--duration', type=float, default=10., help='Observation window in seconds (1-300)')
    p.add_argument('--config', type=Path, help='Private JSON configuration; never commit this file')
    p.add_argument('--host', help='SSH host or alias; overrides config, never the UDP target')
    p.add_argument('--key', help='Optional local SSH key; otherwise use SSH config/agent')
    p.add_argument('--audit', type=Path, help='New LOCAL JSONL file; required for real trial')
    p.add_argument('--execute', action='store_true', help='Enable real manual/auto requests; otherwise no motion packets')
    p.add_argument('--fall-policy', choices=['any', 'natural-only'], default='any',
                   help='Auto: any includes observed soft stops; natural-only is currently observation-only (cause source unverified)')
    p.add_argument('--interactive', action='store_true', help='Observe first, then require the displayed one-time terminal phrase; dry-run unless --execute')
    p.add_argument('--operator-ready', action='store_true')
    p.add_argument('--clearance-confirmed', action='store_true')
    return p


def bootstrap(options):
    root = Path(__file__).resolve().parent
    script = ['import sys, types', 'sys.dont_write_bytecode = True']
    for name in ('recovery_config', 'recovery_core', 'recovery_runtime', 'recovery_auto'):
        source = (root/(name+'.py')).read_text()
        script += [f'm = types.ModuleType({name!r})', f'sys.modules[{name!r}] = m',
                   f'exec(compile({source!r}, {name!r}, "exec"), m.__dict__)']
    script += [f'sys.modules["recovery_runtime"].configure({options.get("config", {})!r})',
               f'raise SystemExit(sys.modules["recovery_runtime"].run({options!r}))']
    return '\n'.join(script)+'\n'


class TerminalRelay:
    """Forward operator intent only for the displayed current server offer."""
    def __init__(self, direction, clock=time.monotonic):
        self.direction, self.clock = direction, clock
        self.offer = None
        self.deadline = 0.

    def receive(self, event):
        if event.get('event') == 'confirmation_ready':
            duration = event.get('expires_in_s', 0.)
            if (event.get('direction') != self.direction or not isinstance(duration, (int, float))
                    or not math.isfinite(duration) or not 0 < duration <= 15.):
                self.offer = None
                return
            self.offer = event
            self.deadline = self.clock()+duration
        elif (event.get('event') == 'confirmation_invalidated' and self.offer
                and event.get('nonce') == self.offer.get('nonce')):
            self.offer = None
        elif event.get('event') in ('operator_confirmed', 'stopped', 'finished', 'confirmation_dry_run_complete'):
            self.offer = None

    def submit(self, text):
        if text == 'CANCEL':
            self.offer = None
            return {'action': 'cancel'}
        offer, self.offer = self.offer, None
        if offer is None or self.clock() >= self.deadline:
            raise ValueError('当前没有有效口令；请等待终端显示确认框，提前输入不会排队执行。')
        if text != offer.get('phrase'):
            raise ValueError('口令不匹配；没有发送确认。等待新的口令。')
        return dict(action='confirm', nonce=offer['nonce'], waiting_event=offer['waiting_event'],
                    direction=offer['direction'], phrase=offer['phrase'])


class InteractiveDisplay:
    """Human prompts on the terminal; the complete JSON stream stays in audit.

    Receiving/validating telemetry never pauses while a prompt is displayed.
    Only the display is quiet: expiry/invalidation and stop conditions continue.
    """
    REASONS = {
        'need_stable_fallen_orientation': '尚未确认稳定倒地',
        'need_new_factory_recovery_wait_event': '等待新的原厂恢复等待事件（重启工具不会沿用旧事件）',
        'factory_still_reports_normal_standing': '原厂仍报告正常站立',
        'soft_emergency_observed': '已记录软急停，需要本次现场恢复确认',
        'factory_fault_observed': '原厂有错误记录，本次不能继续',
        'native_diagnostics_missing_or_stale': '遥测尚未就绪或已过期',
        'native_protocol_not_verified': '遥测协议尚未核实',
        'native_clock_not_ready': '遥测时钟尚未就绪',
        'native_state_stale': '原厂状态已过期',
        'native_diagnostics_invalid': '遥测诊断尚未就绪',
        'need_exactly_one_native_telemetry_publisher': '遥测发布者数量异常',
        'battery_not_above_20_percent': '电量条件不满足',
        'imu_source_timestamp_stale_or_invalid': 'IMU源时间无效或已过期，不能积累倒地时间',
    }

    def __init__(self, direction, clock=time.monotonic):
        self.direction = '左' if direction == 'left' else '右'
        self.clock = clock
        self.waiting_nonce = None
        self.last_reasons = None
        self.last_status = -math.inf

    def handle(self, event):
        kind = event.get('event')
        if kind == 'observing':
            mode = '真实实验' if event.get('execute') else '演练：不会发送动作'
            print(f'\n【{mode} · 向机器狗自身{self.direction}侧恢复】\n'
                  '正在观察。条件满足后，这里会显示一个确认框。\n'
                  '就在当前终端输入；CANCEL 加回车可取消，Ctrl+C 可退出。', flush=True)
        elif kind == 'observation':
            # Do not scroll the confirmation away or interleave ordinary status
            # text with the operator's keyboard echo. Full data is still saved.
            if self.waiting_nonce is not None:
                return
            reasons = tuple(dict.fromkeys(event.get('reasons', [])+event.get('factory_reasons', [])))
            if reasons == self.last_reasons or self.clock()-self.last_status < 5.:
                return
            self.last_reasons, self.last_status = reasons, self.clock()
            labels = [self.REASONS.get(r, r) for r in reasons]
            print('\n[等待] '+('；'.join(labels) if labels else '正在核对现场确认条件'), flush=True)
        elif kind == 'fall_candidate_alert':
            print(f"\n[已检测到倒地] 机身倾角约 {event.get('tilt_deg', 0.):.1f}°。等待恢复确认条件。", flush=True)
        elif kind == 'confirmation_ready':
            self.waiting_nonce = event['nonce']
            label = '真实动作请求' if not event.get('dry_run', True) else '仅演练，不发送动作'
            seconds = max(0, math.ceil(event.get('expires_in_s', 0.)))
            stop = ('本次有软急停记录；输入此口令表示你在现场明确请求恢复。'
                    if event.get('resume_soft_stop') else '输入此口令表示你在现场明确请求恢复。')
            print('\n'+'='*64+'\n'
                  f'【等待你输入 · {label} · 向{self.direction}翻身】\n'
                  '请确认：机器人已倒地，所选方向空地和急停条件满足。\n'
                  +stop+'\n\n'
                  '请在当前终端，原样输入下面这一整行，再按 Enter：\n\n'
                  f"    {event['phrase']}\n\n"
                  f'口令有效期最多 {seconds} 秒；过期会撤销或显示新口令。\n'
                  '无需另开终端，也无需等到 $ 提示符。CANCEL 加回车可取消。\n'
                  +'='*64+'\n'
                  '现在可在下面输入：', flush=True)
        elif kind == 'confirmation_invalidated':
            if event.get('nonce') == self.waiting_nonce:
                self.waiting_nonce = None
                print('\n[口令已失效] 不会使用旧口令发起请求；请等待新的确认框。', flush=True)
        elif kind in ('local_confirmation_rejected', 'confirmation_rejected'):
            self.waiting_nonce = None
            detail = event.get('error', '') if kind == 'local_confirmation_rejected' else '确认未生效，请等待新的确认框。'
            print('\n[未确认] '+detail, flush=True)
        elif kind == 'operator_confirmed':
            self.waiting_nonce = None
            print('\n[已收到确认] 正在重新检查当前状态。', flush=True)
        elif kind == 'confirmation_dry_run_complete':
            self.waiting_nonce = None
            print('\n[演练完成] 已验证确认流程，没有发送动作。真实实验需要新的现场确认。', flush=True)
        elif kind == 'direction_sent':
            print('\n[请求进行中] 已发送方向设置，正在复核状态。', flush=True)
        elif kind == 'stand_sent':
            print('\n[已发送一次恢复请求] 正在观察原厂反馈。', flush=True)
        elif kind == 'factory_success_and_upright_observed':
            print('\n[原厂报告恢复成功] 已观察到直立姿态，请现场确认实际结果。', flush=True)
        elif kind == 'factory_fault':
            self.waiting_nonce = None
            print('\n[原厂错误记录] 本次恢复确认已阻止，请先检查错误原因。', flush=True)
        elif kind in ('finished', 'stopped'):
            self.waiting_nonce = None
            detail = event.get('error')
            suffix = '此前已尝试发送，原厂动作可能继续。' if event.get('sent_motion_packets') else '没有发送动作。'
            print('\n[会话已结束] '+suffix+(('\n原因：'+detail) if detail else ''), flush=True)


class AutoDisplay:
    """Compact automatic countdown; full telemetry is retained in the audit."""
    def __init__(self):
        self.last_status = None

    def handle(self, event):
        kind = event.get('event')
        if kind == 'auto_configuration':
            mode = '真实自动恢复' if not event.get('dry_run', True) else '只观察，不发送动作'
            policy = ('不区分倒地原因，包含已有软急停'
                      if event['fall_policy'] == 'any' else '仅自然摔倒：来源未核实，发送保持禁用')
            print(f'\n【{mode} · 稳定倒地5秒后向左恢复】\n{policy}\n'
                  'CANCEL 加回车或 Ctrl+C 可退出；已触发的原厂动作可能继续。', flush=True)
        elif kind == 'auto_observation':
            reasons = tuple(dict.fromkeys(event.get('reasons', [])+event.get('factory_reasons', [])))
            held = int(event.get('stable_fallen_s', 0.))
            status = (event.get('phase'), held, reasons)
            if status == self.last_status:
                return
            self.last_status = status
            labels = [InteractiveDisplay.REASONS.get(r, r) for r in reasons]
            if event.get('valid_imu_callbacks') == 0 and event.get('tilt_deg') is None:
                text = '尚未收到IMU姿态数据，无法检测倒地；请先检查只读遥测接收器'
            elif event.get('phase') == 'recovering':
                text = '正在观察原厂恢复结果'
            elif event.get('phase') == 'waiting_for_normal_standing':
                text = '等待正常站立稳定后重新允许下一次倒地'
            else:
                text = f'稳定倒地 {held}/5 秒'
                if labels:
                    text += '；'+'；'.join(labels)
            print('\n[自动] '+text, flush=True)
        elif kind == 'telemetry_unavailable':
            print('\n[遥测未就绪，停止自动模式] '+event['hint'], flush=True)
        elif kind == 'auto_countdown_reset':
            print('\n[计时重置] 检测到新的软急停，重新观察完整5秒。', flush=True)
        elif kind == 'auto_trigger_ready':
            print('\n[自动条件满足] '+('仅记录，不发送。' if event['dry_run'] else '重新检查后请求一次左翻身。'), flush=True)
        elif kind == 'auto_dry_run_ready':
            print('\n[观察验证完成] 未发送运动指令，未创建尝试记录。', flush=True)
        elif kind == 'stand_sent':
            print('\n[已发送一次恢复请求] 不重复发送。', flush=True)
        elif kind == 'auto_recovery_complete':
            print('\n[恢复完成] 原厂报告成功，已观察到正常站立状态和稳定直立姿态。', flush=True)
        elif kind == 'auto_rearmed':
            print('\n[已重新等待] 可以检测下一次倒地。', flush=True)
        elif kind in ('factory_fault', 'stopped', 'finished'):
            suffix = '已发送过请求，原厂动作可能继续。' if event.get('sent_motion_packets') else '没有发送运动指令。'
            print('\n[自动会话结束] '+suffix+'\n'+event.get('error', ''), flush=True)
        elif kind in ('auto_input_ignored', 'local_confirmation_rejected'):
            print('\n[输入未生效] 自动模式无需口令；输入 CANCEL 加回车可退出。', flush=True)


def stream_session(process, audit, interactive, direction, automatic=False):
    relay = TerminalRelay(direction)
    display = AutoDisplay() if automatic else (InteractiveDisplay(direction) if interactive else None)
    output_buffer, input_buffer = b'', b''
    output_fd = process.stdout.fileno()
    input_fd = sys.stdin.fileno() if interactive or (automatic and sys.stdin.isatty()) else None

    def record(line):
        if audit:
            audit.write(line+'\n')
            audit.flush()
            os.fsync(audit.fileno())
        if not interactive and not automatic:
            print(line, flush=True)

    def output(line):
        text = line.decode('utf8', 'replace')
        record(text)
        try:
            event = json.loads(text)
        except ValueError:
            return
        if not isinstance(event, dict):
            return
        if not automatic:
            relay.receive(event)
        if display:
            display.handle(event)

    while True:
        fds = [output_fd]+([input_fd] if input_fd is not None else [])
        ready, _, _ = select.select(fds, [], [], .2)
        # Drain status first, so an available invalidation beats keyboard input.
        if output_fd in ready:
            data = os.read(output_fd, 65536)
            if not data:
                if output_buffer:
                    output(output_buffer)
                return process.wait()
            output_buffer += data
            if len(output_buffer) > 262144:
                raise RuntimeError('Remote output exceeded framing limit')
            lines = output_buffer.split(b'\n')
            output_buffer = lines.pop()
            for line in lines:
                output(line)
        if input_fd in ready:
            data = os.read(input_fd, 4096)
            if not data:
                process.stdin.close()
                input_fd = None
                continue
            input_buffer += data
            if len(input_buffer) > 4096:
                raise RuntimeError('Terminal input exceeded framing limit')
            lines = input_buffer.split(b'\n')
            input_buffer = lines.pop()
            for line in lines:
                try:
                    message = relay.submit(line.decode('utf8').strip())
                except (ValueError, UnicodeDecodeError) as exc:
                    event = dict(event='local_confirmation_rejected', error=str(exc))
                    record(json.dumps(event, ensure_ascii=False))
                    display.handle(event)
                    # Invalidate the server offer too. This can never confirm it.
                    message = {'action': 'reject'}
                record(json.dumps(dict(event='local_operator_input', message=message), ensure_ascii=False))
                process.stdin.write((json.dumps(message)+'\n').encode())
                process.stdin.flush()


def robot_shell_path(value):
    """Quote the suffix while allowing ONLY the robot's HOME expansion."""
    return ('"$HOME"/'+shlex.quote(value[2:])) if value.startswith('~/') else shlex.quote(value)


def remote_command(config, launcher):
    commands = ['source '+robot_shell_path(config['ros_setup']),
                'source '+robot_shell_path(config['robot_workspace']+'/install/setup.bash'),
                'export PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID='+str(config['ros_domain_id'])+' ROS_LOCALHOST_ONLY=0']
    commands += (['export FASTRTPS_DEFAULT_PROFILES_FILE='+robot_shell_path(config['dds_profile'])]
                 if config['dds_profile'] else ['unset FASTRTPS_DEFAULT_PROFILES_FILE'])
    commands.append('exec python3 -u -c '+shlex.quote(launcher))
    return ' && '.join(commands)


def main(argv=None):
    p = parser()
    args = p.parse_args(argv)
    try:
        config = load_config(args.config, args.host, args.key)
    except (OSError, ValueError) as exc:
        p.error('Invalid configuration: '+str(exc))
    args.host, args.key = config['ssh_host'], config['ssh_key']
    if args.mode == 'auto':
        args.direction = args.direction or 'left'
        if args.direction != 'left':
            p.error('Auto mode currently supports the validated left direction only')
    if not 1 <= args.duration <= 300:
        p.error('Duration must be finite and between 1 and 300 seconds')
    if args.mode in ('plan', 'manual') and args.direction is None:
        p.error('Choose --direction left or right explicitly')
    if args.execute and args.mode != 'auto':
        args.interactive = True
    if args.interactive and args.mode != 'manual':
        p.error('--interactive is only available in manual mode')
    if args.execute and args.mode != 'auto' and (args.mode != 'manual' or not args.audit or not all(
            (args.operator_ready, args.clearance_confirmed))):
        p.error('Real trial needs manual, new --audit, --operator-ready, --clearance-confirmed and a later terminal confirmation')
    if args.interactive and not sys.stdin.isatty():
        p.error('Interactive confirmation requires a real terminal; piped/preset confirmation is not accepted')
    if args.mode == 'auto' and args.execute:
        if not args.audit:
            p.error('Auto execution requires a new --audit file')
        if args.fall_policy == 'natural-only':
            p.error('Natural-only execution is unavailable until a reliable fall cause source is verified; omit --execute to observe')
    if args.mode == 'plan':
        print(json.dumps(dict(profile=PROFILE, ssh_host=args.host,
                              robot_local_target=[config['factory_ip'], config['factory_port']],
                              direction=args.direction, requests=plan(args.direction),
                              packet_hex=[v.hex() for v in packets(args.direction)],
                              sends_packets=False, automatic_execution_available=False), indent=2))
        return 0
    if not args.host or not config['factory_ip']:
        p.error('Remote modes require ssh_host/--host and factory_ip in --config; see the configuration example')
    options = dict(mode=args.mode, direction=args.direction, duration=args.duration,
                   execute=args.execute, operator_ready=args.operator_ready,
                   clearance_confirmed=args.clearance_confirmed, interactive=args.interactive,
                   fall_policy=args.fall_policy, audit_enabled=args.audit is not None,
                   config={k: v for k, v in config.items() if k not in ('ssh_host', 'ssh_key')})
    # Source is streamed into Python memory on the robot, not installed there.
    # Reuse the running receive-only bridge and domain; never restart services.
    # -c keeps SSH stdin available for confirmation/cancellation. The only
    # payload is our local source, safely encoded and shell-quoted; no code is
    # installed on the robot and no passwords enter this channel.
    source = base64.b64encode(bootstrap(options).encode()).decode('ascii')
    launcher = 'import base64;exec(compile(base64.b64decode('+repr(source)+'), "recovery_bootstrap", "exec"))'
    command = remote_command(config, launcher)
    argv = ['ssh']+(['-i', str(Path(args.key).expanduser())] if args.key else [])+['-o', 'IdentitiesOnly=yes', '-o', 'BatchMode=yes',
            '-o', 'StrictHostKeyChecking=yes', '-o', 'ConnectTimeout=5',
            '-o', 'ServerAliveInterval=2', '-o', 'ServerAliveCountMax=2',
            args.host, 'bash -c '+shlex.quote(command)]
    audit = process = None
    try:
        if args.audit:
            audit = args.audit.open('x', encoding='utf8')
            audit.write(json.dumps(dict(event='local_start', **options, ssh_host=args.host))+'\n')
            audit.flush()
            os.fsync(audit.fileno())
        process = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        if not args.interactive and args.mode != 'auto':
            process.stdin.close()
        if args.mode == 'auto':
            return stream_session(process, audit, False, args.direction, automatic=True)
        return stream_session(process, audit, args.interactive, args.direction)
    except (OSError, RuntimeError, KeyboardInterrupt) as exc:
        print('Recovery tool stopped: '+(str(exc) or 'interrupted')+'. No retry is performed.', file=sys.stderr)
        return 2
    finally:
        if process is not None:
            if process.stdin and not process.stdin.closed:
                process.stdin.close()
            # Closing the control pipe cancels a pending confirmation remotely.
            # Never wait indefinitely for SSH after Ctrl+C or an audit error.
            try:
                process.wait(timeout=2.)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=1.)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            if process.stdout:
                process.stdout.close()
        if audit:
            audit.close()


if __name__ == '__main__':
    raise SystemExit(main())
