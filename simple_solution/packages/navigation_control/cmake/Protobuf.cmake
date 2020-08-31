find_package(Protobuf REQUIRED)

function(protobuf_generate_cpp_with_descriptor
           protobuf_SOURCES
           protobuf_HEADERS
           protobuf_DESCRIPTORS
         )
  cmake_parse_arguments(protobuf "" "EXPORT_MACRO;IMPORT_DIRS;DESTINATION" "" ${ARGN})

  set(PROTO_FILES "${protobuf_UNPARSED_ARGUMENTS}")

  if(NOT PROTO_FILES)
    message(SEND_ERROR "Error: PROTOBUF_GENERATE_CPP() called without any proto files")
    return()
  endif()

  if(NOT protobuf_DESTINATION)
    set(protobuf_DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
  endif()
  file(MAKE_DIRECTORY ${protobuf_DESTINATION})

  if(protobuf_EXPORT_MACRO)
    set(DLL_EXPORT_DECL "dllexport_decl=${protobuf_EXPORT_MACRO}:")
  endif()

  set(${protobuf_SOURCES})
  set(${protobuf_HEADERS})
  set(${protobuf_DESCRIPTORS})

  if(PROTOBUF_GENERATE_CPP_APPEND_PATH)
    # Create an include path for each file specified
    foreach(FIL ${PROTO_FILES})
      get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
      get_filename_component(ABS_PATH ${ABS_FIL} PATH)
      list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
      if(${_contains_already} EQUAL -1)
          list(APPEND _protobuf_include_path -I ${ABS_PATH})
      endif()
    endforeach()
  else()
    set(_protobuf_include_path -I ${CMAKE_CURRENT_SOURCE_DIR})
  endif()

  foreach(DIR ${protobuf_IMPORT_DIRS})
    get_filename_component(ABS_PATH ${DIR} ABSOLUTE)
    list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
    if(${_contains_already} EQUAL -1)
        list(APPEND _protobuf_include_path -I ${ABS_PATH})
    endif()
  endforeach()

  foreach(FIL ${PROTO_FILES})
    get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
    get_filename_component(FIL_WE ${FIL} NAME_WE)
    if(NOT PROTOBUF_GENERATE_CPP_APPEND_PATH)
      get_filename_component(FIL_DIR ${FIL} DIRECTORY)
      if(FIL_DIR)
        set(FIL_WE "${FIL_DIR}/${FIL_WE}")
      endif()
    endif()

    set(_protobuf_protoc_src "${protobuf_DESTINATION}/${FIL_WE}.pb.cc")
    set(_protobuf_protoc_hdr "${protobuf_DESTINATION}/${FIL_WE}.pb.h")
    list(APPEND ${protobuf_SOURCES} "${_protobuf_protoc_src}")
    list(APPEND ${protobuf_HEADERS} "${_protobuf_protoc_hdr}")

    set(_protobuf_protoc_desc "${protobuf_DESTINATION}/${FIL_WE}.desc")
    set(_protobuf_protoc_flags
        "--include_imports"
        "--descriptor_set_out=${_protobuf_protoc_desc}")
    list(APPEND ${protobuf_DESCRIPTORS} "${_protobuf_protoc_desc}")

    add_custom_command(
      OUTPUT "${_protobuf_protoc_src}"
             "${_protobuf_protoc_hdr}"
             ${_protobuf_protoc_desc}
      COMMAND  protobuf::protoc
               "--cpp_out=${DLL_EXPORT_DECL}${protobuf_DESTINATION}"
               ${_protobuf_protoc_flags}
               ${_protobuf_include_path} ${ABS_FIL}
      DEPENDS ${ABS_FIL} protobuf::protoc
      COMMENT "Running C++ protocol buffer compiler on ${FIL}"
      VERBATIM )
  endforeach()

  # Odd that you need to re-set these to get parent scope on them
  # (i.e. can't be done above when the variables are initially defined)
  set(${protobuf_SOURCES}     "${${protobuf_SOURCES}}"     PARENT_SCOPE)
  set(${protobuf_HEADERS}     "${${protobuf_HEADERS}}"     PARENT_SCOPE)
  set(${protobuf_DESCRIPTORS} "${${protobuf_DESCRIPTORS}}" PARENT_SCOPE)

endfunction()
