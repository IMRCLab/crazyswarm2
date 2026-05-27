import importlib


def test_import():
    module = importlib.import_module('crazyflie_py')
    assert module is not None
