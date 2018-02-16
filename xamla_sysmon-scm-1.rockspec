package = "xamla_sysmon"
version = "scm-1"

source = {
  url = "git://github.com/Xamla/xamla_sysmon"
}

description = {
  summary = "Xamla System Monitor",
  detailed = [[
  ]],
  homepage = "",
  license = "proprietary"
}

dependencies = {
"torch >= 7.0"
}

build = {
type = "command",
  build_command = [[
  cmake -E make_directory build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$(PREFIX)" && $(MAKE) -j4
  ]],
    install_command = "cd build && $(MAKE) install"
}
