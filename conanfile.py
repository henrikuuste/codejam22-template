from importlib.metadata import requires
from conans import ConanFile, CMake


class PathplanningConan(ConanFile):
    name = "pathplanning"
    version = "0.1.0"
    license = "MIT"
    author = "Henri Kuuste"
    url = "<Package recipe repository url here, for issues about the package>"
    description = "<Description of Pathplanning here>"
    settings = "os", "compiler", "build_type", "arch"
    options = {"shared": [True, False], "fPIC": [True, False]}
    default_options = {"shared": False, "fPIC": True}
    generators = ("cmake", "cmake_find_package_multi")
    exports_sources = ["lib/*", "docs/*", "cmake/*"]
    requires = ( "spdlog/1.9.2" )

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

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
