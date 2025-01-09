# Copyright: (C) 2022 Istituto Italiano di Tecnologia | Robotics, Brains and Cognitive Science
# Authors: Letícia Berto
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname motivatedAutonomousAgent_SIM)

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.xml
  ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.txt ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.sh ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.csv)
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template  )

yarp_install(FILES ${conf} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
