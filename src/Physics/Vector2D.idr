module Physics.Vector2D

public export
Vector2D : Type
Vector2D = (Double, Double)

export
Num Vector2D where
  (a1, b1) + (a2, b2) = (a1+a2, b1+b2)
  (a1, b1) * (a2, b2) = (a1*a2, b1*b2)
  fromInteger a = (cast a, cast a)

export
Neg Vector2D where
  negate (x, y) = (-x, -y)
  (x2, y2) - (x1, y1) = (x2-x1, y2-y1)

export
Abs Vector2D where
  abs (x, y) = (abs x, abs y)

export
norm : Vector2D -> Double
norm (x, y) = sqrt $ x*x + y*y

export
scale : Double -> Vector2D -> Vector2D
scale a (x, y) = (a*x, a*y)

infix 7 `scale`

public export
Cast (Integer, Integer) Vector2D where
  cast (x, y) = (the Double (cast x), the Double (cast y))

public export
Cast Vector2D (Int, Int) where
  cast (x, y) = (cast x, cast y)

export
nullVector : Vector2D
nullVector = (0, 0)

export
magnitude : Vector2D -> Double
magnitude (a, b) = sqrt(a*a + b*b)

export
normed : Vector2D -> Vector2D
normed x = let magnitude' = 1.0/(magnitude x) in magnitude' `scale` x

export
angle : Vector2D -> Double
angle (x, y) = atan2 y x
