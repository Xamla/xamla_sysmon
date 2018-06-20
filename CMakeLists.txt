cmake_minimum_required (VERSION 2.8)
project (xamlasysmon)
find_package(Torch REQUIRED)

install(DIRECTORY "lua/" DESTINATION "${Torch_INSTALL_LUA_PATH_SUBDIR}/xamla_sysmon" FILES_MATCHING PATTERN "*.lua")
