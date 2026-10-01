# Adding custom sources

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
  bmi08a.c
  bmi08g.c
  bmi08xa.c
  bmi088.c
  bmp5.c
  bmp581.c
  CD-PA1616S.c
  dumb_timer.c
  orientation_estimator.c
  us_timer.c
  )