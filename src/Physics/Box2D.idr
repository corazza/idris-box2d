module Physics.Box2D

import Control.ST

import Physics.Vector2D

%link C "box2d.o"
%include C "box2d.h"
%flag C "-lBox2D -lstdc++"

export
data Body = MkBody Ptr

export
data World = MkWorld Ptr


-- public export
-- interface Box2DIO (m : Type -> Type) where
--   getPosition : (body : Body) -> m Vector2D
--   getAngle : (body : Body) -> m Double
--
-- implementation Box2DIO IO where
--   getPosition (MkBody ptr) = do
--     x <- foreign FFI_C "getPosx" (Ptr -> IO Double) ptr
--     y <- foreign FFI_C "getPosy" (Ptr -> IO Double) ptr
--     pure (x, y)
--
--   getAngle (MkBody ptr) =
--     foreign FFI_C "getAngle" (Ptr -> IO Double) ptr

public export
interface Box2DPhysics (m : Type -> Type) where
  SBox2D : Type

  createWorld : (gravity : Vector2D) -> ST m Var [add SBox2D]
  destroyWorld : (sbox : Var) -> ST m () [remove sbox SBox2D]

  createGroundBody : (sbox : Var) ->
                     (position : Vector2D) ->
                     (dimensions : Vector2D) ->
                     ST m Body [sbox ::: SBox2D]
  -- createBody : (sbox : Var) -> (def : BodyDef) -> ST m Body [sbox ::: SBox2D]

  createBox : (sbox : Var) ->
              (position : Vector2D) ->
              (dimensions : Vector2D) ->
              (angle : Double) ->
              (density : Double) -> (friction : Double) ->
              ST m Body [sbox ::: SBox2D]

  step : (box : Var) ->
         (timeStep : Double) ->
         (velocityIterations : Int) ->
         (positionIterations : Int) ->
         ST m () [box ::: SBox2D]

  applyImpulse : (box : Var) -> Body -> Vector2D -> ST m () [box ::: SBox2D]

  getMass : (body : Body) -> m Double
  getPosition : (body : Body) -> m Vector2D
  getAngle : (body : Body) -> m Double

export
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

  createGroundBody sbox (posx, posy) (dimx, dimy) = with ST do
    MkWorld world <- read sbox
    ptr <- lift $ foreign FFI_C "createGroundBody"
                          (Ptr -> Double -> Double -> Double -> Double -> IO Ptr)
                          world posx posy dimx dimy
    pure $ MkBody ptr

  createBox box (posx, posy) (dimx, dimy) angle density friction = with ST do
    MkWorld world <- read box
    ptr <- lift $ foreign FFI_C "createBox"
                          (Ptr -> Double -> Double -> Double -> Double ->
                           Double -> Double -> Double -> IO Ptr)
                          world posx posy dimx dimy angle density friction
    pure $ MkBody ptr

  step box ts vel pos = with ST do
    MkWorld world <- read box
    lift $ foreign FFI_C "step" (Ptr -> Double -> Int -> Int -> IO ())
                   world ts vel pos

  applyImpulse box (MkBody body) (a, b) = with ST do
    MkWorld world <- read box
    lift $ foreign FFI_C "applyImpulse" (Ptr -> Ptr -> Double -> Double -> IO ())
                   world body a b

  getMass (MkBody ptr) = foreign FFI_C "getMass" (Ptr -> IO Double) ptr >>= pure

  getPosition (MkBody ptr) = do
    x <- foreign FFI_C "getPosx" (Ptr -> IO Double) ptr
    y <- foreign FFI_C "getPosy" (Ptr -> IO Double) ptr
    pure (x, y)

  getAngle (MkBody ptr) =
    foreign FFI_C "getAngle" (Ptr -> IO Double) ptr
