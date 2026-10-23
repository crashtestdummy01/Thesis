# ============================================================
# Remote Library Fetching
#
# Verbatim from the bota_driver_cpp demo repo's cmake/FetchBotaLibraries.cmake.
# ============================================================

function(get_latest_version BASE_URL OUT_VERSION)
    set(INDEX_PATH "${CMAKE_BINARY_DIR}/index_tmp.html")
    message(STATUS "[Bota Libraries] Checking latest version from ${BASE_URL}/index.html ...")
    file(DOWNLOAD "${BASE_URL}/index.html" "${INDEX_PATH}")
    file(READ "${INDEX_PATH}" INDEX_CONTENT)
    string(REGEX MATCH "Latest Release \\(v([0-9]+\\.[0-9]+\\.[0-9]+)\\)" _ "${INDEX_CONTENT}")
    if(CMAKE_MATCH_1 STREQUAL "")
        message(FATAL_ERROR "[Bota Libraries] Could not parse version from ${BASE_URL}/index.html")
    endif()
    set(${OUT_VERSION} "${CMAKE_MATCH_1}" PARENT_SCOPE)
    file(REMOVE "${INDEX_PATH}")
endfunction()

function(fetch_library LIB_NAME VERSION BASE_URL ZIP_NAME)
    set(LIB_DEST "${LIBS_DIR}/${LIB_NAME}")
    set(VERSION_FILE "${LIB_DEST}/VERSION")

    if(EXISTS "${VERSION_FILE}")
        file(READ "${VERSION_FILE}" INSTALLED_VERSION)
        string(STRIP "${INSTALLED_VERSION}" INSTALLED_VERSION)
        if(INSTALLED_VERSION STREQUAL "${VERSION}")
            message(STATUS "[Bota Libraries] ${LIB_NAME} v${VERSION} already up to date, skipping.")
            return()
        else()
            message(STATUS "[Bota Libraries] ${LIB_NAME} version changed (${INSTALLED_VERSION} -> ${VERSION}), re-downloading...")
            file(REMOVE_RECURSE "${LIB_DEST}")
        endif()
    endif()

    set(ZIP_PATH "${CMAKE_BINARY_DIR}/${ZIP_NAME}")
    message(STATUS "[Bota Libraries] Downloading ${LIB_NAME} v${VERSION} for ${PLATFORM}...")
    file(DOWNLOAD
        "${BASE_URL}/${ZIP_NAME}"
        "${ZIP_PATH}"
        STATUS DOWNLOAD_STATUS
    )
    list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
    if(NOT STATUS_CODE EQUAL 0)
        list(GET DOWNLOAD_STATUS 1 ERROR_MSG)
        message(FATAL_ERROR "[Bota Libraries] Failed to download ${LIB_NAME}: ${ERROR_MSG}")
    endif()

    message(STATUS "[Bota Libraries] Extracting ${LIB_NAME}...")
    set(EXTRACT_TMP "${CMAKE_BINARY_DIR}/${LIB_NAME}_tmp")
    file(REMOVE_RECURSE "${EXTRACT_TMP}")
    file(ARCHIVE_EXTRACT INPUT "${ZIP_PATH}" DESTINATION "${EXTRACT_TMP}")
    file(REMOVE "${ZIP_PATH}")

    file(GLOB CPP_CONTENTS "${EXTRACT_TMP}/cpp/binaries/*")
    file(MAKE_DIRECTORY "${LIB_DEST}")
    foreach(ITEM ${CPP_CONTENTS})
        file(COPY "${ITEM}" DESTINATION "${LIB_DEST}")
    endforeach()
    file(REMOVE_RECURSE "${EXTRACT_TMP}")

    file(WRITE "${VERSION_FILE}" "${VERSION}")
    message(STATUS "[Bota Libraries] ${LIB_NAME} v${VERSION} ready.")
endfunction()