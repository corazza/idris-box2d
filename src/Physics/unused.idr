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
