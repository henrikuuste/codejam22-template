from importlib.metadata import requires
from conans import ConanFile, CMake
from conans.tools import load
import re

def get_version():
    try:
        content = load("lib/CMakeLists.txt")
        version = re.search("pathplanning(?:\s+)VERSION (.*)(?:\s+)LANGUAGES", content).group(1)
        return version.strip()
    except Exception as e:
        print(e)
        return None

def get_license():
    try:
        content = load("LICENSE")
        license = re.search(r"^(.*)$", content, re.MULTILINE).group(1)
        return license.strip()
    except Exception as e:
        print(e)
        return ""
    
def get_required():
  try:
    content = load("cmake/conan_setup.cmake")
    libs = re.search(r"\(LIB_DEPS\s*([^\)]*)", content, re.DOTALL).group(1)
    if not libs:
      return []
    # TODO remove comments?
    libs = re.compile(r"\s+").split(libs)
    return libs
  except Exception as e:
      print(e)
      return []

def get_build_required():
  try:
    content = load("cmake/conan_setup.cmake")
    libs = re.search(r"\(LIB_BUILD_DEPS\s*([^\)]*)", content, re.DOTALL).group(1)
    if not libs:
      return []
    # TODO remove comments?
    libs = re.compile(r"\s+").split(libs)
    return libs
  except Exception as e:
      print(e)
      return []

class PathplanningConan(ConanFile):
    name = "pathplanning"
    version = get_version()
    license = get_license()
    author = "Henri Kuuste <henri.kuuste@gmail.com>"
    url = "https://github.com/henrikuuste/codejam22-template"
    description = "Template project for CodeJam22 pathplanning library"
    settings = "os", "compiler", "build_type", "arch"
    options = {"shared": [True, False]}
    default_options = {"shared": False}
    generators = ("cmake", "cmake_find_package_multi")
    exports = ["LICENSE", "cmake/*", "lib/CMakeLists.txt"]
    exports_sources = ["lib/*", "docs/*", "cmake/*", "LICENSE"]
    requires = get_required()
    build_requires = get_build_required()

    def configure_cmake(self):
        cmake = CMake(self)
        cmake.configure(source_folder="lib")
        return cmake

    def build(self):
        cmake = self.configure_cmake()
        cmake.build()

    def package(self):
        cmake = self.configure_cmake()
        cmake.install()

    def package_info(self):
        self.cpp_info.libs = ["pathplanning"]
