module Physics.Box2D

import Control.ST

import Physics.Vector2D

%link C "box2d.o"
%include C "box2d.h"
%flag C "-lBox2D -lstdc++"

export
test : IO Int
test = foreign FFI_C "hello" (IO Int)

public export
data Shape = Box Double Double

public export
record FixtureDef where
  constructor MkFixtureDef
  shape : Shape
  density : Double
  friction : Double

public export
data BodyType = Dynamic | Static

public export
record BodyDef where
  constructor MkBodyDef
  position : Vector2D
  type : BodyType
  fixture : FixtureDef

export
data Body = MkBody Ptr

export
data World = MkWorld Ptr

interface Box2DPhysics (m : Type -> Type) where
  SBox2D : Type

  createWorld : (gravity : Vector2D) -> ST m Var [add SBox2D]
  destroyWorld : (sbox : Var) -> ST m () [remove sbox SBox2D]

implementation Box2DPhysics IO where
  SBox2D = State Box2D.World

  createWorld (x, y) = with ST do
    ptr <- lift $ foreign FFI_C "createWorld" (Double -> Double -> IO Ptr) x y
    sbox2d <- new $ MkWorld ptr
    pure sbox2d

  destroyWorld sbox = with ST do
    MkWorld world <- read sbox
    lift $ foreign FFI_C "destroyWorld" (Ptr -> IO ()) world
    delete sbox;
