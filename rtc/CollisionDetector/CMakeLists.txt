set(seq_dir ${PROJECT_SOURCE_DIR}/rtc/SequencePlayer)
set(comp_sources ${seq_dir}/interpolator.cpp CollisionDetector.cpp CollisionDetectorService_impl.cpp GLscene.cpp FCLLinkPair.cpp)

pkg_check_modules(fcl fcl REQUIRED)

add_definitions(-DQHULL)
include_directories(${LIBXML2_INCLUDE_DIR} ${QHULL_INCLUDE_DIR} ${seq_dir} ${fcl_INCLUDE_DIR})
add_library(CollisionDetector SHARED ${comp_sources})
target_link_libraries(CollisionDetector hrpsysUtil ${QHULL_LIBRARIES} ${fcl_LIBRARIES})
set_target_properties(CollisionDetector PROPERTIES PREFIX "")

add_executable(CollisionDetectorComp CollisionDetectorComp.cpp ${comp_sources} )
target_link_libraries(CollisionDetectorComp hrpsysUtil ${QHULL_LIBRARIES} ${fcl_LIBRARIES})

#add_executable(SetupCollisionPair SetupCollisionPair.cpp)
#target_link_libraries(SetupCollisionPair CollisionDetector ${OPENHRP_LIBRARIES} ${QHULL_LIBRARIES})

add_executable(CollisionDetectorViewer CollisionDetectorViewer.cpp GLscene.cpp)
target_link_libraries(CollisionDetectorViewer hrpsysUtil)
set_target_properties (CollisionDetectorViewer PROPERTIES COMPILE_DEFINITIONS "USE_COLLISION_STATE")

#set(target CollisionDetector CollisionDetectorComp SetupCollisionPair CollisionDetectorViewer)
set(target CollisionDetector CollisionDetectorComp CollisionDetectorViewer)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
