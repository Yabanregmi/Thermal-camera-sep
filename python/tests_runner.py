import unittest
import os
import sys
import importlib
from datetime import datetime
from pathlib import Path
from typing import List

def get_next_report_number(directory: Path, prefix_pattern: str = r"(\d+)_test_report_") -> int:
    files: List[str] = os.listdir(directory)
    import re
    pattern = re.compile(prefix_pattern + r"\d{2}_\d{2}_\d{4}_\d{2}_\d{2}_\d{2}\.txt")
    numbers = [int(m.group(1)) for f in files if (m := pattern.match(f))]
    return max(numbers) + 1 if numbers else 1

if __name__ == "__main__":
    suite = unittest.TestSuite()
    test_dir = Path(__file__).parent / "tests"
    sys.path.insert(0, str(Path(__file__).parent))

    testmodules = []
    for file in os.listdir(test_dir):
        if file.startswith("test_") and file.endswith(".py"):
            modulename = f"tests.{file[:-3]}"
            try:
                mod = importlib.import_module(modulename)
                loaded = unittest.defaultTestLoader.loadTestsFromModule(mod)
                if loaded.countTestCases() > 0:
                    suite.addTests(loaded)
                    testmodules.append(modulename)
            except Exception as e:
                print(f"Warnung: Modul {modulename} konnte nicht geladen werden: {e}")

    report_dir = test_dir / "testreport"
    os.makedirs(report_dir, exist_ok=True)
    timestamp_str = datetime.now().strftime("%d_%m_%Y_%H_%M_%S")
    report_num = get_next_report_number(report_dir)
    filename = f"{report_num:05d}_test_report_{timestamp_str}.txt"
    filepath = os.path.join(report_dir, filename)

    with open(filepath, "w") as file:
        file.write("========================================\n")
        file.write("Testbericht (file)\n")
        file.write(f"Datum und Uhrzeit: {timestamp_str}\n")
        file.write("========================================\n\n")
        file.write("Geladene Testmodule:\n")
        for mod in testmodules:
            file.write(f"- {mod}\n")
        file.write("\n")
        if suite.countTestCases() > 0:
            runner_file = unittest.TextTestRunner(stream=file, verbosity=2)
            runner_file.run(suite)
        else:
            file.write("Keine Testmodule geladen oder keine Tests gefunden.\n")