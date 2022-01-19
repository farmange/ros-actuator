
# The `RCUTILS_CONSOLE_OUTPUT_FORMAT` environment variable can be used to set
#  * the output format of messages logged to the console.
#  * Available tokens are:
#  *   - `file_name`, the full file name of the caller including the path
#  *   - `function_name`, the function name of the caller
#  *   - `line_number`, the line number of the caller
#  *   - `message`, the message string after it has been formatted
#  *   - `name`, the full logger name
#  *   - `severity`, the name of the severity level, e.g. `INFO`
#  *   - `time`, the timestamp of log message in floating point seconds
#  *   - `time_as_nanoseconds`, the timestamp of log message in integer nanoseconds

export RCUTILS_CONSOLE_OUTPUT_FORMAT="{time} {name} [{severity}]: {message}"
#export ROSCONSOLE_FORMAT='${logger} [${severity}]: ${message}'

export RCUTILS_COLORIZED_OUTPUT=1  # 1 for forcing it
