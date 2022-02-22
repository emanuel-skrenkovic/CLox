#!/usr/bin/python3

import os
import sys
import glob
import inspect
import subprocess

from dataclasses import dataclass

tests_path = "tests/"
executable = "bin/main"


@dataclass
class TestStatus:
    exit_code: int
    stdout: str
    stderr: str


def test(func):
    func.is_test = True
    return func


def run_test(test_file):
    process = subprocess.run([executable, tests_path + test_file], check=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    return TestStatus(process.returncode, str(process.stdout), str(process.stderr))


def get_tests():
    for f in inspect.getmembers(sys.modules[__name__], inspect.isfunction):
        func = f[1]
        if hasattr(func, 'is_test') and func.is_test:
            yield f


@test
def test_let():
    result = run_test("let.lox")
    assert result.exit_code != 0, result.stderr


@test
def test_var():
    result = run_test("var.lox")
    assert result.exit_code == 0, result.stderr


@test
def test_switch():
    result = run_test("switch.lox")
    assert result.exit_code == 0, result.stderr


@test
def test_for():
    result = run_test("for.lox")
    assert result.exit_code == 0, result.stderr


@test
def test_assignment():
    result = run_test("assignment.lox")
    assert result.exit_code == 0, result.stderr


@test
def test_assignment_get_constant():
    result = run_test("assignment_get_constant.lox")
    assert result.exit_code == 0, result.stderr


@test
def test_continue_for():
    result = run_test("continue_for.lox")
    assert result.exit_code == 0, result.stderr


@test
def test_continue_while():
    result = run_test("continue_while.lox")
    assert result.exit_code == 0, result.stderr


@test
def test_function():
    result = run_test("function.lox")
    assert result.exit_code == 0, result.stderr


@test
def test_function_stack_trace():
    result = run_test("function_stack_trace.lox")
    assert result.exit_code == 70, result.stderr


@test
def test_function_native():
    result = run_test("function_native.lox")
    assert result.exit_code == 0, result.stderr


if __name__ == "__main__":
    for test in get_tests():
        test_name = test[0]
        test_func = test[1]

        failed = False

        try:
            test_result = test_func()
            print("[INFO]  \'{}\' -> \x1b[1;32;40m Success! \x1b[0m".format(test_name))
        except AssertionError as error:
            if failed != True:
                failed = True
            print("\x1b[1;31;40m[ERROR] \x1b[0m\'{}\'-> {}".format(test_name, error))

    sys.exit(failed)
