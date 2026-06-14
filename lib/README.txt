Vendored GNU GLPK 5.0
======================

Files:
  glpk-5.0.tar.gz   — official source archive
  glpk-5.0/         — extracted source tree

Source: https://ftp.gnu.org/gnu/glpk/glpk-5.0.tar.gz
License: GPLv3 (see glpk-5.0/COPYING)

Build static library (optional, for linking lp_solver):
  cd glpk-5.0
  ./configure --prefix="$PWD/../install" --disable-shared
  make -j
  make install

Headers: install/include/glpk.h
Library: install/lib/libglpk.a

Link lp_solver against install/lib/libglpk.a with -DLP_HAVE_GLPK when the
GLPK backend in src/lp_solve_glpk.c is implemented.
