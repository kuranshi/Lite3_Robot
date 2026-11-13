import importlib.util
from pathlib import Path


def scanner():
    path = Path(__file__).resolve().parents[1]/'scripts/check_export.py'
    spec = importlib.util.spec_from_file_location('export_check', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_release_content_is_clean():
    check = scanner()
    assert check.findings(check.release_files()) == []


def test_personal_paths_network_addresses_and_private_files_are_detected(tmp_path):
    path = tmp_path/'robot.local.json'
    path.write_text('/'+'home'+'/example-user/project '+'.'.join(['192','168','123','9']))
    reasons = [label for _, label in scanner().findings([path])]
    assert 'absolute personal home' in reasons
    assert 'private network address' in reasons
    assert 'private file or runtime artifact' in reasons


def test_credentials_are_detected_without_echoing_them(tmp_path):
    path = tmp_path/'example.txt'
    path.write_text('-----BEGIN '+'OPENSSH '+'PRIVATE KEY-----\n'+'ghp_'+'a'*40)
    reasons = [label for _, label in scanner().findings([path])]
    assert 'private key material' in reasons and 'GitHub token' in reasons


def test_documentation_addresses_and_generic_home_expansion_are_allowed(tmp_path):
    path = tmp_path/'example.txt'
    path.write_text('192.0.2.10 ~/jy_exe robot contributors@example.invalid')
    assert scanner().findings([path]) == []
