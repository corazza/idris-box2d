module Physics.Box2D

import Data.Vect
import Control.ST

import Physics.Vector2D
import public Physics.Box2D.Definitions
import Physics.Box2D.Defaults

%link C "box2d.o"
%include C "box2d.h"
%flag C "-lBox2D -lstdc++"

export
data Body = MkBody Ptr

export
data World = MkWorld Ptr

export
data Fixture = MkFixture Ptr

public export
record CollisionForBody where
  constructor MkCollisionForBody
  id : Int
  body : Body
  velocity : Vector2D

public export
data Event = CollisionStart CollisionForBody CollisionForBody
           | CollisionStop CollisionForBody CollisionForBody

export
createWorld : Vector2D -> IO Box2D.World
createWorld (x, y) = foreign FFI_C "createWorld" (Double -> Double -> IO Ptr) x y >>= pure . MkWorld

export
destroyWorld : Box2D.World -> IO ()
destroyWorld (MkWorld ptr) = foreign FFI_C "destroyWorld" (Ptr -> IO ()) ptr

maybeBoolToInt : Maybe Bool -> Int
maybeBoolToInt (Just True) = 1
maybeBoolToInt _ = 0

typeToInt : BodyType -> Int
typeToInt Static = 0
typeToInt Dynamic = 1
typeToInt Kinematic = 2

export
createBody : Box2D.World -> BodyDefinition -> IO (Int, Body)
createBody (MkWorld ptr) (MkBodyDefinition type (x, y) angle fixedRotation bullet)
  = with IO do
      body <- foreign FFI_C "createBody" (Ptr -> Int -> Double -> Double -> Double -> Int -> Int -> IO Ptr)
                      ptr (typeToInt type) x y (fromMaybe 0 angle) (maybeBoolToInt fixedRotation)
                      (maybeBoolToInt bullet)
      id <- foreign FFI_C "getId" (Ptr -> IO Int) body
      pure (id, MkBody body)

export
createFixture : Body -> FixtureDefinition -> IO Fixture
createFixture (MkBody ptr) (MkFixtureDefinition shape offset angle density friction restitution)
  = let (offx, offy) = fromMaybe nullVector offset
        angle' = fromMaybe 0 angle
        density' = fromMaybe Defaults.density density
        friction' = fromMaybe Defaults.friction friction
        restitution' = fromMaybe Defaults.restitution restitution in case shape of
          Circle r => foreign FFI_C "createFixtureCircle"
            (Ptr -> Double -> Double -> Double -> Double -> Double -> Double -> Double -> IO Ptr)
            ptr r offx offy angle' density' friction' restitution' >>= pure . MkFixture
          Box (x, y) => foreign FFI_C "createFixtureBox"
            (Ptr -> Double -> Double -> Double -> Double -> Double -> Double -> Double -> Double -> IO Ptr)
            ptr x y offx offy angle' density' friction' restitution' >>= pure . MkFixture
          Polygon xs => ?createPolygonShapeFixture

export
createFixture' : Body -> Shape -> (density' : Double) -> IO Fixture
createFixture' body shape density'
  = let fixture = record {density = Just density'} (defaultFixture shape) in
        createFixture body fixture

export
destroy : Box2D.World -> Body -> IO ()
destroy (MkWorld ptr_w) (MkBody ptr_b) = foreign FFI_C "destroy" (Ptr -> Ptr -> IO ()) ptr_w ptr_b

export
step : Box2D.World ->
       (timeStep : Double) ->
       (velocityIterations : Int) ->
       (positionIterations : Int) ->
       IO ()
step (MkWorld ptr) = foreign FFI_C "step" (Ptr -> Double -> Int -> Int -> IO ()) ptr

export
applyImpulse : Body -> Vector2D -> IO ()
applyImpulse (MkBody ptr) (x, y)
  = foreign FFI_C "applyImpulse" (Ptr -> Double -> Double -> IO ()) ptr x y

export
applyForce : Body -> (force : Vector2D) -> (offset : Vector2D) -> IO ()
applyForce (MkBody ptr) (x, y) (offx, offy)
  = foreign FFI_C "applyForce" (Ptr -> Double -> Double -> Double -> Double -> IO ())
            ptr x y offx offy

export
getMass : Body -> IO Double
getMass (MkBody ptr) = foreign FFI_C "getMass" (Ptr -> IO Double) ptr

export
getPosition : Body -> IO Vector2D
getPosition (MkBody ptr) = with IO do
  x <- foreign FFI_C "getPosx" (Ptr -> IO Double) ptr
  y <- foreign FFI_C "getPosy" (Ptr -> IO Double) ptr
  pure (x, y)

export
getWorldCenter : Body -> IO Vector2D
getWorldCenter (MkBody ptr) = with IO do
  x <- foreign FFI_C "getWCx" (Ptr -> IO Double) ptr
  y <- foreign FFI_C "getWCy" (Ptr -> IO Double) ptr
  pure (x, y)

export
getAngle : Body -> IO Double
getAngle (MkBody ptr) = foreign FFI_C "getAngle" (Ptr -> IO Double) ptr

export
getVelocity : Body -> IO Vector2D
getVelocity (MkBody ptr) = do
  x <- foreign FFI_C "getVelx" (Ptr -> IO Double) ptr
  y <- foreign FFI_C "getVely" (Ptr -> IO Double) ptr
  pure (x, y)

export
pollEvent : Box2D.World -> IO (Maybe Event)
pollEvent (MkWorld w_ptr) = with IO do
  ptr <- foreign FFI_C "topCollision" (Ptr -> IO Ptr) w_ptr
  if ptr == null then pure Nothing else with IO do
    ptr_body_one <- foreign FFI_C "getBodyOne" (Ptr -> IO Ptr) ptr
    ptr_body_two <- foreign FFI_C "getBodyTwo" (Ptr -> IO Ptr) ptr
    id_one <- foreign FFI_C "getIdOne" (Ptr -> IO Int) ptr
    id_two <- foreign FFI_C "getIdTwo" (Ptr -> IO Int) ptr
    start <- foreign FFI_C "getStart" (Ptr -> IO Int) ptr
    velx_one <- foreign FFI_C "getColVelx" (Ptr -> Int -> IO Double) ptr 1
    velx_two <- foreign FFI_C "getColVelx" (Ptr -> Int -> IO Double) ptr 2
    vely_one <- foreign FFI_C "getColVely" (Ptr -> Int -> IO Double) ptr 1
    vely_two <- foreign FFI_C "getColVely" (Ptr -> Int -> IO Double) ptr 2
    foreign FFI_C "popCollision" (Ptr -> IO ()) w_ptr
    let for_one = MkCollisionForBody id_one (MkBody ptr_body_one) (velx_one, vely_one)
    let for_two = MkCollisionForBody id_two (MkBody ptr_body_two) (velx_two, vely_two)
    let event = if start == 1
      then CollisionStart for_one for_two
      else CollisionStop for_one for_two
    pure $ Just event

-- pollEvents : (box : Var) -> ST m (List Event) [box ::: SBox2D]
export
pollEvents : Box2D.World -> IO (List Event)
pollEvents world = with IO do
  Just event <- pollEvent world | pure []
  pure $ event :: !(pollEvents world)
