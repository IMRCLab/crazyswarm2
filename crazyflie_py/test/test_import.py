import importlib
import unittest


class TestImport(unittest.TestCase):

    def test_import(self):
        module = importlib.import_module('crazyflie_py')
        self.assertIsNotNone(module)
