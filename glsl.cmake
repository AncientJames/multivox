file(READ "${INPUT_FILE}" CONTENT)
string(REPLACE "\\" "\\\\" CONTENT "${CONTENT}")
string(REPLACE "\"" "\\\"" CONTENT "${CONTENT}")
string(REPLACE "\n" "\\n\"\n\"" CONTENT "${CONTENT}")
get_filename_component(VARNAME "${INPUT_FILE}" NAME_WE)
string(REPLACE "." "_" VARNAME "${VARNAME}")
file(WRITE "${OUTPUT_FILE}" "static const char* ${VARNAME} = \"${CONTENT}\";\n")
