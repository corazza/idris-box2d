module Physics.Box2D

import Data.Vect
import Control.ST

import public Physics.Vector2D
import public Physics.Box2D.Definitions
import Physics.Box2D.Defaults

%link C "box2d.o"
%include C "box2d.h"
%flag C "-lBox2D -lstdc++"

export
data World = MkWorld Ptr

public export
record CollisionForBody where
  constructor MkCollisionForBody
  id : Int
  body : Body
  velocity : Vector2D

public export
data AABB = MkAABB Vector2D Vector2D -- lower, upper

public export
data AABBQuery = MkAABBQuery Int AABB -- query_id,

-- public export
-- data QueryResult = MkQueryResult Int Body

public export
data Event = CollisionStart CollisionForBody CollisionForBody
           | CollisionStop CollisionForBody CollisionForBody
           | QueryResult Int Int Body

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
createFixture (MkBody ptr) fd
  = let (offx, offy) = fromMaybe nullVector (offset fd)
        angle' = fromMaybe 0 (angle fd)
        groupIndex' = fromMaybe 0 (groupIndex fd)
        categoryBits' = fromMaybe 0x0001 (categoryBits fd)
        maskBits' = fromMaybe 0xFFFF (maskBits fd)
        density' = fromMaybe Defaults.density (density fd)
        friction' = fromMaybe Defaults.friction (friction fd)
        restitution' = fromMaybe Defaults.restitution (restitution fd) in case shape fd of
          Circle r => foreign FFI_C "createFixtureCircle"
            (Ptr -> Double -> Double -> Double -> Double -> Double -> Double -> Double ->
             Int -> Int -> Int -> IO Ptr)
            ptr r offx offy angle' density' friction' restitution'
            groupIndex' categoryBits' maskBits' >>= pure . MkFixture
          Box (x, y) => foreign FFI_C "createFixtureBox"
            (Ptr -> Double -> Double -> Double -> Double -> Double -> Double -> Double -> Double ->
             Int -> Int -> Int -> IO Ptr)
            ptr x y offx offy angle' density' friction' restitution'
            groupIndex' categoryBits' maskBits' >>= pure . MkFixture
          Polygon xs => ?createPolygonShapeFixture

export
createFixture' : Body -> Shape -> (density' : Double) -> IO Fixture
createFixture' body shape density'
  = let fixture = record {density = Just density'} (defaultFixture shape) in
        createFixture body fixture

defaultCollide : Maybe Bool -> Int
defaultCollide Nothing = 0
defaultCollide (Just False) = 0
defaultCollide (Just True) = 1

export
createRevoluteJoint : Box2D.World -> RevoluteJointDefinition -> IO Joint
createRevoluteJoint (MkWorld world_ptr) def
  = let collideConnected = defaultCollide (collideConnected def)
        MkBody bodyA_ptr = bodyA def
        MkBody bodyB_ptr = bodyB def
        (localAnchorAx, localAnchorAy) = localAnchorA def
        (localAnchorBx, localAnchorBy) = localAnchorB def
        in with IO do
          joint_ptr <- foreign FFI_C "createRevoluteJoint"
                (Ptr -> Ptr -> Ptr -> Int -> Double -> Double -> Double -> Double -> IO Ptr)
                world_ptr bodyA_ptr bodyB_ptr collideConnected
                localAnchorAx localAnchorAy localAnchorBx localAnchorBy
          pure $ MkJoint joint_ptr

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
queryAABB : Box2D.World -> AABBQuery -> IO ()
queryAABB (MkWorld ptr) (MkAABBQuery id (MkAABB (lx, ly) (ux, uy)))
  = foreign FFI_C "queryAABB" (Ptr -> Int -> Double -> Double -> Double -> Double -> IO ())
              ptr id lx ly ux uy

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
pollCollision : Box2D.World -> IO (Maybe Event)
pollCollision (MkWorld w_ptr) = with IO do
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
pollCollisions : Box2D.World -> IO (List Event)
pollCollisions world = with IO do
  Just event <- pollCollision world | pure []
  pure $ event :: !(pollCollisions world)

-- export
pollQuery : Box2D.World -> IO (Maybe Event)
pollQuery (MkWorld w_ptr) = with IO do
  ptr <- foreign FFI_C "topQueryResult" (Ptr -> IO Ptr) w_ptr
  if ptr == null then pure Nothing else with IO do
    query_id <- foreign FFI_C "getQueryId" (Ptr -> IO Int) ptr
    body_id <- foreign FFI_C "getQueryBodyId" (Ptr -> IO Int) ptr
    body <- foreign FFI_C "getQueryBody" (Ptr -> IO Ptr) ptr
    foreign FFI_C "popQuery" (Ptr -> IO ()) w_ptr
    pure $ Just $ QueryResult query_id body_id (MkBody body)

-- export
pollQueries : Box2D.World -> IO (List Event)
pollQueries world = with IO do
  Just result <- pollQuery world | pure []
  pure $ result :: !(pollQueries world)

export
pollEvents : Box2D.World -> IO (List Event)
pollEvents world = pure $ !(pollCollisions world) ++ !(pollQueries world)
