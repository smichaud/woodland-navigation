/* \author Bastian Steder - steder@informatik.uni-freiburg.de */

/////////////////////////////////////////////////////////////////////////
float MyPcl::NarfssKeypoint::getDescriptorDistance(const float* descriptor, const float* descriptor2, int descriptor_size) {
  float descriptor_distance = 0.0f;
  for (int i=0; i<descriptor_size; ++i)
    descriptor_distance += fabsf(descriptor[i]-descriptor2[i]);
    //descriptor_distance += std::min(0.1f, fabsf(descriptor[i]-descriptor2[i]));
  return descriptor_distance / descriptor_size;
}

/////////////////////////////////////////////////////////////////////////
void MyPcl::NarfssKeypoint::blurDescriptor(float* descriptor, int descriptor_size) const {
  // Blur descriptor
  float blur_angle = pcl::deg2rad(15.0f);
  int blur_radius = pcl_lrintf(descriptor_size*blur_angle/pcl::deg2rad(360.0f));
  float tmp_descriptor[descriptor_size];
  for (int descriptor_value_idx=0; descriptor_value_idx<descriptor_size; ++descriptor_value_idx)
  {
    float& value = tmp_descriptor[descriptor_value_idx];
    value = 0.0f;
    for (int descriptor_value_idx2=descriptor_value_idx-blur_radius; descriptor_value_idx2<=descriptor_value_idx+blur_radius; ++descriptor_value_idx2)
    {
      int index2 = (descriptor_value_idx2<0 ? descriptor_size+descriptor_value_idx2 :
                                                (descriptor_value_idx2>=descriptor_size ?
                                                  descriptor_value_idx2-descriptor_size : descriptor_value_idx2));
      value += descriptor[index2];
    }
    value /= float(2*blur_radius+1);
  }
  std::copy(tmp_descriptor, tmp_descriptor+descriptor_size, descriptor);
}
