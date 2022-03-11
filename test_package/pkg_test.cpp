#include <pathplanning/pathplanning.h>
#include <spdlog/spdlog.h>

int main() {
  spdlog::info("Library version {}", pathplanning::version());
  pathplanning::plan();
}
