

template <typename DrawObjectType>
MultiContainerObject<DrawObjectType>::MultiContainerObject(const typename std::vector<DrawObjectType>* objects) :
  ALUGLViewerObject(), _objects(objects)
{
  setDrawColor(0.0f, 0.0f, 0.8f);
}

template <typename DrawObjectType>
MultiContainerObject<DrawObjectType>::~MultiContainerObject() {
}

template <typename DrawObjectType>
void MultiContainerObject<DrawObjectType>::draw() const
{
  if (!_objects)
    return;

  for(typename std::vector<DrawObjectType>::const_iterator it = _objects->begin(); it != _objects->end(); it++)
  {
    (*it)->draw();
  }
}

template <typename DrawObjectType>
void MultiContainerObject<DrawObjectType>::drawPoints() const
{

}

