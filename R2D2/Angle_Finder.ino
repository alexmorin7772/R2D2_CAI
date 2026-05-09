float angle_finder(int x_value) {
  return atan(static_cast<float>(x_value - 160) / FOCAL_LENGTH);
}
