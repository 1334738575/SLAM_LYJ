#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace py = pybind11;
namespace fs = std::filesystem;

namespace {

fs::path ResolvePythonExecutable(const fs::path& venvPath) {
    const std::vector<fs::path> candidates = {
        venvPath / "python.exe",
        venvPath / "Scripts" / "python.exe",
    };

    for (const auto& candidate : candidates) {
        if (fs::exists(candidate)) {
            return candidate;
        }
    }

    throw std::runtime_error("python executable was not found in the environment");
}

fs::path GetVenvSitePackages(const fs::path& venvPath) {
    return venvPath / "Lib" / "site-packages";
}

void ValidateInputs(
    const fs::path& venvPath,
    const fs::path& moduleFile,
    const std::string& functionName) {
    ResolvePythonExecutable(venvPath);

    if (!fs::exists(moduleFile)) {
        throw std::runtime_error("python module file not found");
    }

    if (moduleFile.extension() != ".py") {
        throw std::runtime_error("module file must be a .py file");
    }

    if (functionName.empty()) {
        throw std::runtime_error("function name must not be empty");
    }
}

void PrepareVenvForImports(const fs::path& venvPath, const fs::path& moduleFile) {
    const fs::path pythonExe = ResolvePythonExecutable(venvPath);
    const fs::path scriptsDir = venvPath / "Scripts";
    const fs::path libraryBinDir = venvPath / "Library" / "bin";

//#ifdef _WIN32
//    const std::wstring venv = venvPath.wstring();
//
//    _wputenv_s(L"VIRTUAL_ENV", venv.c_str());
//    _wputenv_s(L"CONDA_PREFIX", venv.c_str());
//    _wputenv_s(L"PYTHONHOME", venv.c_str());
//
//    const wchar_t* oldPath = _wgetenv(L"PATH");
//    std::wstring combinedPath = venvPath.wstring();
//    if (fs::exists(scriptsDir)) {
//        combinedPath += L";";
//        combinedPath += scriptsDir.wstring();
//    }
//    if (fs::exists(libraryBinDir)) {
//        combinedPath += L";";
//        combinedPath += libraryBinDir.wstring();
//    }
//    if (oldPath != nullptr && oldPath[0] != L'\0') {
//        combinedPath += L";";
//        combinedPath += oldPath;
//    }
//    _wputenv_s(L"PATH", combinedPath.c_str());
//    _wputenv_s(L"PYTHONEXECUTABLE", pythonExe.wstring().c_str());
//#endif

    py::module_ os = py::module_::import("os");
    os.attr("environ")["VIRTUAL_ENV"] = py::str(venvPath.string());
    os.attr("environ")["CONDA_PREFIX"] = py::str(venvPath.string());
    os.attr("environ")["PYTHONHOME"] = py::str(venvPath.string());
    os.attr("environ")["PYTHONEXECUTABLE"] = py::str(ResolvePythonExecutable(venvPath).string());

    py::module_ site = py::module_::import("site");
    const fs::path sitePackages = GetVenvSitePackages(venvPath);
    if (fs::exists(sitePackages)) {
        site.attr("addsitedir")(py::str(sitePackages.string()));
    }

    py::module_ sys = py::module_::import("sys");
    sys.attr("path").attr("insert")(0, py::str(moduleFile.parent_path().string()));

    try {
        os.attr("add_dll_directory")(py::str(venvPath.string()));
    } catch (const py::error_already_set&) {
    }

    try {
        if (fs::exists(scriptsDir)) {
            os.attr("add_dll_directory")(py::str(scriptsDir.string()));
        }
    } catch (const py::error_already_set&) {
    }

    try {
        if (fs::exists(libraryBinDir)) {
            os.attr("add_dll_directory")(py::str(libraryBinDir.string()));
        }
    } catch (const py::error_already_set&) {
    }
}

py::object ImportModuleFromFile(const fs::path& moduleFile) {
    py::module_ importlib_util = py::module_::import("importlib.util");
    py::module_ sys = py::module_::import("sys");

    const std::string moduleName = moduleFile.stem().string();
    py::object spec = importlib_util.attr("spec_from_file_location")(
        py::str(moduleName),
        py::str(moduleFile.string()));

    if (spec.is_none()) {
        throw std::runtime_error("failed to create module spec");
    }

    py::object module = importlib_util.attr("module_from_spec")(spec);
    sys.attr("modules")[py::str(moduleName)] = module;

    py::object loader = spec.attr("loader");
    if (loader.is_none()) {
        throw std::runtime_error("module loader is missing");
    }

    loader.attr("exec_module")(module);
    return module;
}

std::string RunPythonFunctionInVenv(
    const fs::path& venvPath,
    const fs::path& moduleFile,
    const std::string& functionName,
    const std::vector<std::string>& args) {
    ValidateInputs(venvPath, moduleFile, functionName);

    const fs::path pythonExe = ResolvePythonExecutable(venvPath);
    const fs::path scriptsDir = venvPath / "Scripts";
    const fs::path libraryBinDir = venvPath / "Library" / "bin";

#ifdef _WIN32
    const std::wstring venv = venvPath.wstring();

    _wputenv_s(L"VIRTUAL_ENV", venv.c_str());
    _wputenv_s(L"CONDA_PREFIX", venv.c_str());
    _wputenv_s(L"PYTHONHOME", venv.c_str());

    const wchar_t* oldPath = _wgetenv(L"PATH");
    std::wstring combinedPath = venvPath.wstring();
    if (fs::exists(scriptsDir)) {
        combinedPath += L";";
        combinedPath += scriptsDir.wstring();
    }
    if (fs::exists(libraryBinDir)) {
        combinedPath += L";";
        combinedPath += libraryBinDir.wstring();
    }
    if (oldPath != nullptr && oldPath[0] != L'\0') {
        combinedPath += L";";
        combinedPath += oldPath;
    }
    _wputenv_s(L"PATH", combinedPath.c_str());
    _wputenv_s(L"PYTHONEXECUTABLE", pythonExe.wstring().c_str());
#endif

    py::scoped_interpreter guard{};
    PrepareVenvForImports(venvPath, moduleFile);

    py::object module = ImportModuleFromFile(moduleFile);
    if (!py::hasattr(module, functionName.c_str())) {
        throw std::runtime_error("target function was not found in module");
    }

    py::object function = module.attr(functionName.c_str());
    py::tuple pyArgs(args.size());
    for (size_t i = 0; i < args.size(); ++i) {
        pyArgs[i] = py::cast(args[i]);
    }

    py::object result = function(*pyArgs);
    return py::str(result).cast<std::string>();
}

}  // namespace

int main(int argc, char* argv[]) {


    try {
        //.\build\Debug\run_python_in_venv.exe F : \AnacondaFile\envs\testEnv D : \testPybind\codex4\sample_module.py greet lyj
        //const fs::path venvPath = "F:/AnacondaFile/envs/testEnv";
        //const fs::path moduleFile = "D:/testPybind/codex4/sample_module.py";
        //const std::string functionName = "greet";
        //std::vector<std::string> args;
        //args.emplace_back("lyj");

        const fs::path venvPath = "F:/AnacondaFile/envs/surfel_splatting";
        const fs::path moduleFile = "D:/2d-gaussian-splatting/train.py";
        const std::string functionName = "run";
        std::vector<std::string> args;
        args.emplace_back("D:/tmp/colmapData/mask/dense");
        args.emplace_back("D:/tmp/colmapData/mask/3DGS2");

        //if (argc < 4) {
        //    std::cerr
        //        << "Usage: run_python_in_venv <venv_path> <module.py> <function_name> [args...]\n"
        //        << "Example: run_python_in_venv C:\\project\\venv C:\\project\\sample_module.py greet Alice\n";
        //    return 1;
        //}
        //const fs::path venvPath = argv[1];
        //const fs::path moduleFile = argv[2];
        //const std::string functionName = argv[3];
        //std::vector<std::string> args;
        //for (int i = 4; i < argc; ++i) {
        //    args.emplace_back(argv[i]);
        //}

        const std::string result = RunPythonFunctionInVenv(venvPath, moduleFile, functionName, args);
        std::cout << "Function result: " << result << '\n';
        return 0;
    } catch (const py::error_already_set& ex) {
        std::cerr << "Python error:\n" << ex.what() << '\n';
        return 1;
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << '\n';
        return 1;
    }
}
