module Physics.Box2D

%link C "box2d.o"
%link C "api.o"
%include C "box2d.h"
%flag C "-lBox2D -lstdc++"

export
test : IO Int
test = foreign FFI_C "test" (IO Int)
