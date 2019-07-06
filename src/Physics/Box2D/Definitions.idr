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

data Shape = Circle Double
           | Box Vector2D
           | Polygon (List Vector2D)
%name Shape shape

record FixtureDefinition where
  constructor MkFixtureDefinition
  shape : Shape
  offset : Maybe Vector2D
  angle : Maybe Double
  density : Maybe Double
  friction : Maybe Double
  restitution : Maybe Double
%name FixtureDefinition fixtureDef

defaultFixture : Shape -> FixtureDefinition
defaultFixture shape
  = MkFixtureDefinition shape Nothing Nothing Nothing Nothing Nothing
