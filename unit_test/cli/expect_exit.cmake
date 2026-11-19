# SPDX-FileCopyrightText: 2026 Sergiu Deitsch
#
# SPDX-License-Identifier: BSD-2-Clause

# Runs the command after "--" and fails unless it exits with the status in
# EXPECTED, which tells a crash or a wrong status apart from the failure that
# the command reports. The files in the optional list ABSENT must not exist
# after the run, which removes them before.

set (_command)
set (_collect FALSE)
math (EXPR _last "${CMAKE_ARGC} - 1")
foreach (_index RANGE ${_last})
  if (_collect)
    list (APPEND _command "${CMAKE_ARGV${_index}}")
  elseif (CMAKE_ARGV${_index} STREQUAL "--")
    set (_collect TRUE)
  endif (_collect)
endforeach (_index)

foreach (_file IN LISTS ABSENT)
  file (REMOVE "${_file}")
endforeach (_file)

execute_process (
  COMMAND ${_command}
  RESULT_VARIABLE _result
  OUTPUT_VARIABLE _output
  ERROR_VARIABLE _output
)

if (NOT _result STREQUAL EXPECTED)
  message (FATAL_ERROR
    "${_command} exited with ${_result} but expected ${EXPECTED}:\n${_output}")
endif (NOT _result STREQUAL EXPECTED)

foreach (_file IN LISTS ABSENT)
  if (EXISTS "${_file}")
    message (FATAL_ERROR "${_command} left ${_file} behind")
  endif (EXISTS "${_file}")
endforeach (_file)
