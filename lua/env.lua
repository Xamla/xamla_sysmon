local ffi = require 'ffi'

local xamla_sysmon = {}


local xamla_sysmon_cdef = [[
  const char* generateMessageText(int status_code);
  int getStatusCode(const char* status_string);
  void start_watch();
  void shutdown_watch();
]]

ffi.cdef(xamla_sysmon_cdef)

xamla_sysmon.lib = ffi.load(package.searchpath('libxamlasysmon', package.cpath))

return xamla_sysmon
