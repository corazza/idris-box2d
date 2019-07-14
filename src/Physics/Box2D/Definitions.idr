module Physics.Box2D.Definition

import Physics.Vector2D
import Physics.Box2D.Defaults

%access public export

-- Maybe values usually indicate optional parameters

data BodyType = Static | Dynamic | Kinematic

Eq BodyType where
  Static == Static = True
  Dynamic == Dynamic = True
  Kinematic == Kinematic = True
  _ == _ = False

record BodyDefinition where
  constructor MkBodyDefinition
  type : BodyType
  position : Vector2D
  angle : Maybe Double
  fixedRotation : Maybe Bool
  bullet : Maybe Bool
%name BodyDefinition bodyDef

Show BodyType where
  show Static = "static"
  show Dynamic = "dynamics"
  show Kinematic = "kinematic"

data Shape = Circle Double
           | Box Vector2D
           | Polygon (List Vector2D)
%name Shape shape

Show Shape where
  show (Circle x) = "circle " ++ show x
  show (Box x) = "box " ++ show x
  show (Polygon xs) = "polygon " ++ show xs

record FixtureDefinition where
  constructor MkFixtureDefinition
  shape : Shape
  offset : Maybe Vector2D
  angle : Maybe Double
  density : Maybe Double
  friction : Maybe Double
  restitution : Maybe Double
%name FixtureDefinition fixtureDef

Show FixtureDefinition where
  show (MkFixtureDefinition shape offset angle density friction restitution)
    =  "{ shape: " ++ show shape
    ++ ", offset: " ++ show offset
    ++ ", angle: " ++ show angle
    ++ ", density: " ++ show density
    ++ ", friction: " ++ show friction
    ++ ", restitution: " ++ show restitution
    ++ " }"

defaultFixture : Shape -> FixtureDefinition
defaultFixture shape
  = MkFixtureDefinition shape Nothing Nothing Nothing Nothing Nothing
