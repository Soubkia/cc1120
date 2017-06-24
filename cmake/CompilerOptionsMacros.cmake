macro (_csi_add_compiler_flags cmake_flags options)
    foreach (flag ${cmake_flags})
        set (new_flags ${${flag}} ${options})
        string (REPLACE ";" " " ${flag} "${new_flags}")
    endforeach ()
endmacro ()

macro (csi_add_cxx_flags)
          _csi_add_compiler_flags ("CMAKE_CXX_FLAGS" "${ARGN}")
endmacro ()

macro (csi_add_c_flags)
     _csi_add_compiler_flags ("CMAKE_C_FLAGS" "${ARGN}")
endmacro ()

macro (csi_add_asm_flags)
     _csi_add_compiler_flags ("CMAKE_ASM_FLAGS" "${ARGN}")
endmacro ()

macro (csi_add_c_cxx_flags)
    csi_add_c_flags (${ARGN})
    csi_add_cxx_flags (${ARGN})
endmacro ()

macro (csi_add_asm_c_cxx_flags)
    csi_add_asm_flags (${ARGN})
    csi_add_c_flags (${ARGN})
    csi_add_cxx_flags (${ARGN})
endmacro ()

macro (csi_set_default_compiler_options)
    if (CMAKE_COMPILER_IS_GNUCC)
        set (extra_c_cxx_flags 
            "-Wall"
            "-Wextra"
            "-Werror"
            "-Wcast-align"
            "-Wcast-qual"
            "-Wmissing-include-dirs"
            "-Wlogical-op"
            "-Wstrict-null-sentinel"
            "-Wredundant-decls"
            "-Wno-unknown-pragmas"
            "-Wundef" 
            "-Wunused"
            "-Wshadow"
            "-fdiagnostics-show-option")
        csi_add_c_cxx_flags (${extra_c_cxx_flags})

        set (extra_cxx_flags
            "-Woverloaded-virtual"
            "-Wno-unused-local-typedefs"
            "-Wctor-dtor-privacy"
            "-Wnoexcept")

        csi_add_cxx_flags (${extra_cxx_flags})
    endif ()
endmacro ()

macro (_csi_add_cpp17_support_direct)
    if (CMAKE_COMPILER_IS_GNUCC OR (CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
        csi_add_cxx_flags("--std=c++1z")
    else ()
        message (WARNING "Unknown compiler, manual set of C++17 support may be required!")
    endif ()
endmacro ()

macro (csi_add_cpp17_support)
    while (TRUE)
        if (CMAKE_VERSION VERSION_LESS 3.1)
            _csi_add_cpp17_support_direct()
            break()
        endif ()

        if ((DEFINED CMAKE_CROSSCOMPILING) AND (CMAKE_VERSION VERSION_LESS 3.6))
            _csi_add_cpp17_support_direct()
            break()
        endif ()

        set (CMAKE_CXX_STANDARD   17)
        set (CMAKE_CXX_EXTENSIONS OFF)
        break()
    endwhile ()
endmacro ()

macro (csi_disable_exceptions)
    if (CMAKE_COMPILER_IS_GNUCC)
        csi_add_cxx_flags("-fno-exceptions -fno-unwind-tables")
    endif ()
endmacro ()

macro (csi_disable_rtti)
    if (CMAKE_COMPILER_IS_GNUCC)
        csi_add_cxx_flags("-fno-rtti")
    endif ()
endmacro ()

macro (csi_disable_stdlib)
    add_definitions(-DNOSTDLIB)
    if (CMAKE_COMPILER_IS_GNUCC)
        csi_add_c_cxx_flags("-nostdlib")
    endif ()
endmacro ()
