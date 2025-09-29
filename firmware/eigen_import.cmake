if (DEFINED EIGEN_PATH)
	message("Using Given EIGEN_PATH '${EIGEN_PATH}')")
else ()
	set(EIGEN_PATH "${CMAKE_CURRENT_LIST_DIR}lib/eigen/")
    message("Using local EIGEN_PATH '${EIGEN_PATH}')")
endif ()

add_library(eigen INTERFACE)


# Add include directory
target_include_directories(eigen INTERFACE 
    ${EIGEN_PATH}
)

# Provide compatibility for projects expecting the imported target Eigen3::Eigen
if(NOT TARGET Eigen3::Eigen)
	add_library(Eigen3::Eigen INTERFACE IMPORTED)
	set_target_properties(Eigen3::Eigen PROPERTIES
		INTERFACE_INCLUDE_DIRECTORIES "${EIGEN_PATH}"
	)
endif()
