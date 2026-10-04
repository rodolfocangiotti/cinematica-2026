import os
import platform
import subprocess
import sys
import time

import psutil

assert (
    platform.system() == "Windows"
), "Ensure these scripts are running on machine running Windows!"


DEBUG = os.environ.get("DEBUG", "true") in ("true", "True", "1")
print(f"DEBUG: {DEBUG}")
print(f"sys.executable: {sys.executable}")


THIS_FILE_DIR = os.path.dirname(os.path.abspath(__file__))

EXPECTED_MAX_PROCESSES = (1, 2)  # Max console has a separate process...
EXPECTED_PYTHON_PROCESSES = (
    4  # One process for this file, and another for communicating with Kinect...
    # Processes in the end are doubled due to venv, check this post on Stack Overflow: https://stackoverflow.com/a/79427366/3837788
)


def _start_max_process() -> None:
    if DEBUG:
        print("Starting Max process...")
    # No return code from these functions...
    # If files are not found, FileNotFoundError exception will be raised!
    os.startfile("run_always.maxpat")
    os.startfile(os.environ.get("MAX_FILE", "dsp_core.maxpat"))
    time.sleep(10.0)
    if DEBUG:
        print("Done.")


def _start_python_processes() -> None:
    if DEBUG:
        print("Starting Python process...")
    res = subprocess.Popen(
        [
            os.path.join(THIS_FILE_DIR, "venv\\Scripts\\python.exe"),
            os.path.join(THIS_FILE_DIR, "kinect_interface.py"),
        ],
        creationflags=subprocess.CREATE_NEW_CONSOLE,
    )
    time.sleep(10.0)
    assert res.returncode is None, res.stderr.decode(
        "ascii"
    )  # There won't be any return code if the process is running correctly...
    if DEBUG:
        print("Done.")


def start_processes() -> None:
    _start_max_process()
    _start_python_processes()


def check_processes() -> None:
    while True:
        # Checking the running processes...
        max_processes = 0
        python_processes = 0
        for p in psutil.process_iter():
            if not p.is_running():
                continue  # Skip terminated processes...

            process_name = p.name()
            if process_name.endswith("Max.exe"):
                if DEBUG:
                    print(f"Found Max process: {p}")
                max_processes += 1
            if process_name.endswith("python.exe"):
                process_wdir = p.cwd()
                if process_wdir == THIS_FILE_DIR:
                    if DEBUG:
                        print(f"Found Python process: {p}")
                    python_processes += 1

        if max_processes not in EXPECTED_MAX_PROCESSES:
            print(
                f"Max processes found: {max_processes}, "
                f"expected: {EXPECTED_MAX_PROCESSES}"
            )
            break
        if python_processes != EXPECTED_PYTHON_PROCESSES:
            print(
                f"Python processes found: {python_processes}, "
                f"expected: {EXPECTED_PYTHON_PROCESSES}"
            )
            break

        if DEBUG:
            print("-" * 32)
        # Wait before executing the new check...
        time.sleep(0.1)

    print("Restarting the service...")


def kill_processes() -> None:
    for p in psutil.process_iter():
        try:
            process_id = p.pid
            process_name = p.name()
            if process_name.endswith("Max.exe"):
                p.terminate()
                print(f"Process {process_name} ({process_id}) has been killed!")
            if process_name.endswith("python.exe"):
                process_cmd = p.cmdline()
                # if os.path.join(THIS_FILE_DIR, "kinect_interface.py") in process_cmd:
                for segment in process_cmd:
                    if segment.endswith("kinect_interface.py"):
                        p.kill()
                        print(f"Process {process_name} ({process_id}) has been killed!")
        except psutil.NoSuchProcess as e:
            print(f"Process {process_id} disappeared ({e}). Skipping it.")


def main():
    try:
        start_processes()
        check_processes()
    except KeyboardInterrupt:
        pass
    kill_processes()


if __name__ == "__main__":
    main()
