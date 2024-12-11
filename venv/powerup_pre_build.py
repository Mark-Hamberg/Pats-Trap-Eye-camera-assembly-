import subprocess
import sys
import os
import time
import importlib
import json
import traceback

def install_package(package):
    return subprocess.check_call([sys.executable, "-m", "pip", "install", package])


module_name = sys.argv[1]
desination_folder = sys.argv[2]
print('Building PowerUp ' + module_name)

try:
    # import pdoc3
    try:
        import pdoc
    except Exception:
        print('Installing pdoc dependency...')
        install_package("pdoc3")
        time.sleep(5)
        import pdoc

        print('pdoc dependency installed.')

    # Docs generation
    print('Generating documentation...')
    html_doc = pdoc.html(module_name, show_source_code=False)
    docs_file_path = desination_folder + os.path.sep + "docs" + os.path.sep + module_name + ".html"
    os.makedirs(desination_folder + os.path.sep + "docs", exist_ok=True)
    doc_file = open(docs_file_path, "wb")
    doc_file.write(html_doc.encode("UTF-8"))
    doc_file.close()
    print("Documentation generated.")

    # import mypi
    try:
        import mypy
    except Exception:
        print('Installing stubgen dependency...')
        install_package("mypy")
        time.sleep(5)
        import mypy

        print('stubgen dependency installed.')

    # Stubs generation
    print('Generating stub...')
    os.makedirs(desination_folder + os.path.sep + "stubs", exist_ok=True)
    subprocess.check_call(["stubgen", "-m", module_name, "-o", desination_folder + os.path.sep + "stubs"])
    print('Stub generated.')

    # Pre-compiled json generation
    try:
        powerup_module = importlib.import_module(module_name)
        _powerup = getattr(powerup_module, "_powerup")
        powerup_properties = _powerup()
        powerup_json = json.dumps(powerup_properties)

        json_file_path = desination_folder + os.path.sep + "config" + os.path.sep + module_name + ".json"
        os.makedirs(desination_folder + os.path.sep + "config", exist_ok=True)
        json_file = open(json_file_path, "wb")
        json_file.write(powerup_json.encode("UTF-8"))
        json_file.close()
    except:
        print('WARNING: PowerUp can not be imported.')

    print('Generating outcome file...')
    outcome_file_path = desination_folder + os.path.sep + "outcome.json"
    json_file = open(outcome_file_path, "wb")
    json_file.write('{"outcome":"OK"}'.encode("UTF-8"))
    json_file.close()
    print('Build completed successfully!')

except:
    traceback.print_exc()
    os.makedirs(desination_folder)
    outcome_file_path = desination_folder + os.path.sep + "outcome.json"
    json_file = open(outcome_file_path, "wb")
    json_file.write('{"outcome":"KO"}'.encode("UTF-8"))
    json_file.close()
    print('Build completed with ERRORS!')


