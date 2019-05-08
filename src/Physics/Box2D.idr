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

public export
data Event = CollisionStart (Int, Body) (Int, Body)
           | CollisionStop (Int, Body) (Int, Body)

export
Show Event where
  show (CollisionStart (id_one, _) (id_two, _)) = "CollisionStart " ++ show id_one ++ " " ++ show id_two
  show (CollisionStop (id_one, _) (id_two, _)) = "CollisionStop " ++ show id_one ++ " " ++ show id_two

public export
interface Box2DPhysics (m : Type -> Type) where
  SBox2D : Type

  createWorld : (gravity : Vector2D) -> ST m Var [add SBox2D]
  destroyWorld : (sbox : Var) -> ST m () [remove sbox SBox2D]

  createWall : (sbox : Var) ->
               (position : Vector2D) ->
               (dimensions : Vector2D) ->
               ST m (Int, Body) [sbox ::: SBox2D]
  -- createBody : (sbox : Var) -> (def : BodyDef) -> ST m Body [sbox ::: SBox2D]

  createBox : (sbox : Var) ->
              (position : Vector2D) ->
              (dimensions : Vector2D) ->
              (angle : Double) ->
              (density : Double) -> (friction : Double) ->
              ST m (Int, Body) [sbox ::: SBox2D]

  step : (box : Var) ->
         (timeStep : Double) ->
         (velocityIterations : Int) ->
         (positionIterations : Int) ->
         ST m () [box ::: SBox2D]

  pollEvent : (box : Var) -> ST m (Maybe Event) [box ::: SBox2D]
  pollEvents : (box : Var) -> ST m (List Event) [box ::: SBox2D]

  applyImpulse : (box : Var) -> Body -> Vector2D -> ST m () [box ::: SBox2D]

  getMass : Body -> m Double
  getPosition : Body -> m Vector2D
  getAngle : Body -> m Double
  getVelocity : Body -> m Vector2D

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

  createWall sbox (posx, posy) (dimx, dimy) = with ST do
    MkWorld world <- read sbox
    ptr <- lift $ foreign FFI_C "createWall"
                          (Ptr -> Double -> Double -> Double -> Double -> IO Ptr)
                          world posx posy dimx dimy
    id <- lift $ foreign FFI_C "getId" (Ptr -> IO Int) ptr
    pure $ (id, MkBody ptr)

  createBox box (posx, posy) (dimx, dimy) angle density friction = with ST do
    MkWorld world <- read box
    ptr <- lift $ foreign FFI_C "createBox"
                          (Ptr -> Double -> Double -> Double -> Double ->
                           Double -> Double -> Double -> IO Ptr)
                          world posx posy dimx dimy angle density friction
    id <- lift $ foreign FFI_C "getId" (Ptr -> IO Int) ptr
    pure $ (id, MkBody ptr)

  step box ts vel pos = with ST do
    MkWorld world <- read box
    lift $ foreign FFI_C "step" (Ptr -> Double -> Int -> Int -> IO ())
                   world ts vel pos

  pollEvent box = with ST do
    ptr <- lift $ foreign FFI_C "topCollision" (IO Ptr)
    if ptr == null then pure Nothing else with ST do
      ptr_body_one <- lift $ foreign FFI_C "getBodyOne" (Ptr -> IO Ptr) ptr
      ptr_body_two <- lift $ foreign FFI_C "getBodyTwo" (Ptr -> IO Ptr) ptr
      id_one <- lift $ foreign FFI_C "getIdOne" (Ptr -> IO Int) ptr
      id_two <- lift $ foreign FFI_C "getIdTwo" (Ptr -> IO Int) ptr
      start <- lift $ foreign FFI_C "getStart" (Ptr -> IO Int) ptr
      lift $ foreign FFI_C "popCollision" (IO ())
      let event = if start == 1
        then CollisionStart (id_one, MkBody ptr_body_one) (id_two, MkBody ptr_body_two)
        else CollisionStop (id_one, MkBody ptr_body_one) (id_two, MkBody ptr_body_two)
      pure $ Just event

  pollEvents box = with ST do
    Just event <- pollEvent box | pure []
    pure $ event :: !(pollEvents box) -- not tail-recursive? probably not a major problem

  applyImpulse box (MkBody body) (a, b) = lift $
    foreign FFI_C "applyImpulse" (Ptr -> Double -> Double -> IO ())
            body a b

  getMass (MkBody ptr) = foreign FFI_C "getMass" (Ptr -> IO Double) ptr >>= pure

  getPosition (MkBody ptr) = do
    x <- foreign FFI_C "getPosx" (Ptr -> IO Double) ptr
    y <- foreign FFI_C "getPosy" (Ptr -> IO Double) ptr
    pure (x, y)

  getAngle (MkBody ptr) =
    foreign FFI_C "getAngle" (Ptr -> IO Double) ptr

  getVelocity (MkBody ptr) = do
    x <- foreign FFI_C "getVelx" (Ptr -> IO Double) ptr
    y <- foreign FFI_C "getVely" (Ptr -> IO Double) ptr
    pure (x, y)
