# idris-box2d
Box2D API for Idris

You need to be able to link with `-lBox2D` and use `#include <Box2D/Box2D.h>`. This most likely necessitates installing Box2D through your distro's repositories, otherwise you might manually need to dabble w/ the include/linkage options throughout this project (`api.h`, Idris directives in `Physics/Box2D.idr`, and stuff in the `Makefile`). [steshaw/idris-sdl2
](https://github.com/steshaw/idris-sdl2) has an example of how to do it properly when `pkg-conf` files are available.
