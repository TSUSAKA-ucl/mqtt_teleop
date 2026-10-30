import os
import sys
import importlib.util

def get_venv_pythonpath_if_has_pyagxarm():
    """
    (1) VIRTUAL_ENV環境変数を調べ
    (2) そのvenvにpyAgxArmがあるか調べ
    (3) 存在すればそのsite-packagesのパスを先頭に加えたPYTHONPATH用の文字列を返す
    """
    # 現在の環境変数をベースにする
    current_env = os.environ.copy()
    current_pythonpath = current_env.get('PYTHONPATH', '')

    # (1) VIRTUAL_ENV 環境変数を取得
    virtual_env = current_env.get('VIRTUAL_ENV')
    if not virtual_env:
        return current_pythonpath  # 仮想環境ではない場合はそのまま

    # Pythonのバージョンを取得 (例: "3.10")
    py_version = f"{sys.version_info.major}.{sys.version_info.minor}"
    
    # site-packages のパスを組み立て
    site_packages_path = os.path.join(virtual_env, 'lib', f'python{py_version}', 'site-packages')

    # (2) その venv 内に pyAgxArm が存在するかチェック
    # (メタデータやディレクトリの存在確認)
    pyagxarm_dir = os.path.join(site_packages_path, 'pyAgxArm')
    
    if os.path.exists(pyagxarm_dir):
        # (3) 存在すれば、既存のPYTHONPATHの先頭に追加
        if current_pythonpath:
            return f"{site_packages_path}:{current_pythonpath}"
        else:
            return site_packages_path

    return current_pythonpath
