add_library(srrg_map_navigator_viewers_library SHARED   
	map_navigator_viewer.cpp map_navigator_viewer.h  
) 

target_link_libraries(srrg_map_navigator_viewers_library   
	${catkin_LIBRARIES}
  	${QGLVIEWER_LIBRARY} 
  	${SRRG_QT_LIBRARIES} 
  	${OPENGL_gl_LIBRARY} 
  	${OPENGL_glu_LIBRARY}   
        ${QT_QTXML_LIBRARY}
        ${QT_QTOPENGL_LIBRARY}
        ${QT_QTGUI_LIBRARY}
        ${QT_QTCORE_LIBRARY}
)

