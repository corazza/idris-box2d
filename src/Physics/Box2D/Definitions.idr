module Physics.Box2D.Definition

import Physics.Vector2D
import Physics.Box2D.Defaults

-- Maybe values usually indicate optional parameters

public export
data Body = MkBody Ptr
public export
data Fixture = MkFixture Ptr
public export
data Joint = MkJoint Ptr

public export
data BodyType = Static | Dynamic | Kinematic

export
Eq BodyType where
  Static == Static = True
  Dynamic == Dynamic = True
  Kinematic == Kinematic = True
  _ == _ = False

public export
record BodyDefinition where
  constructor MkBodyDefinition
  type : BodyType
  position : Vector2D
  angle : Maybe Double
  fixedRotation : Maybe Bool
  bullet : Maybe Bool
%name BodyDefinition bodyDef

export
Show BodyType where
  show Static = "static"
  show Dynamic = "dynamics"
  show Kinematic = "kinematic"

public export
data Shape = Circle Double
           | Box Vector2D
           | Polygon (List Vector2D)
%name Shape shape

export
Show Shape where
  show (Circle x) = "circle " ++ show x
  show (Box x) = "box " ++ show x
  show (Polygon xs) = "polygon " ++ show xs

public export
record FixtureDefinition where
  constructor MkFixtureDefinition
  shape : Shape
  offset : Maybe Vector2D
  angle : Maybe Double
  density : Maybe Double
  friction : Maybe Double
  restitution : Maybe Double
  groupIndex : Maybe Int
  categoryBits : Maybe Int
  maskBits : Maybe Int
%name FixtureDefinition fixtureDef

export
Show FixtureDefinition where
  show fd
    =  "{ shape: " ++ show (shape fd)
    ++ ", offset: " ++ show (offset fd)
    ++ ", angle: " ++ show (angle fd)
    ++ ", density: " ++ show (density fd)
    ++ ", friction: " ++ show (friction fd)
    ++ ", restitution: " ++ show (restitution fd)
    ++ ", group index: " ++ show (groupIndex fd)
    ++ ", category bits: " ++ show (categoryBits fd)
    ++ ", mask bits: " ++ show (maskBits fd)
    ++ " }"

export
defaultFixture : Shape -> FixtureDefinition
defaultFixture shape
  = MkFixtureDefinition shape Nothing Nothing Nothing Nothing Nothing Nothing Nothing Nothing

public export
record RevoluteJointDefinition where
  constructor MkRevoluteJointDefinition
  bodyA : Body
  localAnchorA : Vector2D
  bodyB : Body
  localAnchorB : Vector2D
  collideConnected : Maybe Bool
