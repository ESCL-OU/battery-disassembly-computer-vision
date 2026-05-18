import os


class DirectoryConfig:
    """
    Class for storing directories within the package
    """

    _dir_path = os.path.dirname(os.path.realpath(__file__))
    SCANS_DIR = os.path.join(_dir_path, '..', '..', 'Scans')
    CREALITY_DIR = os.path.join(_dir_path, '..', '..', 'Scans', 'Creality')
    ZIVID_DIR = os.path.join(_dir_path, '..', '..', 'Scans', 'Zivid')
    MODELS_DIR = os.path.join(_dir_path, '..', '..', 'models', 'YOLO')
    ACADOS_MODEL_DIR = os.path.join(_dir_path, '..', '..', 'acados_ocp')
    QUAD_PARAM_DIR = os.path.join(_dir_path, '..', '..', 'quads')
