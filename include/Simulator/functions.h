// <test formation functions>
//

// Returns formation function definition f(x) = 0.
float line(const float x)
{
  return 0.0f;
}
// Returns formation function definition f(x) = x.
float x(const float x)
{
  return x;
}
// Returns formation function definition f(x) = |x|.
float absX(const float x)
{
  return abs(x);
}
// Returns formation function definition f(x) = -0.5 x.
float negHalfX(const float x)
{
  return -0.5f * x;
}
// Returns formation function definition f(x) = -|0.5 x|.
float negAbsHalfX(const float x)
{
  return -abs(0.5f * x);
}
// Returns formation function definition f(x) = -|x|.
float negAbsX(const float x)
{
  return -abs(x);
}
// Returns formation function definition f(x) = x^2.
float parabola(const float x)
{
  return x * x;
  //return pow(x, 2.0f);
}
// Returns formation function definition f(x) = x^3.
float cubic(const float x)
{
  return x * x * x;
  //return pow(x, 3.0f);
}
// Returns formation function definition f(x) = {sqrt(x),  x = 0 | -sqrt|x|, x < 0}.
float condSqrt(const float x)
{
  return sqrt(abs(0.5f * x)) * ((x >= 0) ? 1.0f : -1.0f);
}
// Returns formation function definition f(x) = 0.05 sin(10 x).
float sine(const float x)
{
//  return 0.05f * sin(10.0f * x);
	return sin(x);
}
// Returns formation function definition f(x) = x sqrt(3).
float xRoot3(const float x)
{
  return x * sqrt(3.0f);
}
// Returns formation function definition f(x) = -x sqrt(3).
float negXRoot3(const float x)
{
  return -x * sqrt(3.0f);
}
