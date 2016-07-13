#ifndef NAV_CORE_FOOTPRINT_SET_AWARE
#define NAV_CORE_FOOTPRINT_SET_AWARE

#include <nav_core/footprint_set.h>

namespace nav_core
{
/**
 * @class FootprintSetAware
 * @brief An interface that provices a foootprint set object, a setter and a getter.
 */
class FootprintSetAware
{
public:
  /**
   * @brief Virtual destructor as this will have derived classes
   */
  virtual ~FootprintSetAware()
  {
  }

  /**
   * @brief Sets a pointer to the collection of all footprint sets.
   * @param[in] footprint_set_collection_ptr Pointer to the pointer that tracks the collection of all footprint sets.
   */
  void setFootprintSetCollection(FootprintSetCollection::Ptr* footprint_set_collection_ptr)
  {
    footprint_set_collection_ptr_ = footprint_set_collection_ptr;
  }

protected:
  /**
   * @brief Convenience function that retrieves the active footprint set.
   * @return Pointer to the FootprintSet object.
   */
  FootprintSet::Ptr& footprintMgr()
  {
    return (*footprint_set_collection_ptr_)->getActiveSet();
  }

  /**
   * @brief Convenience function that retireves the collection of all footprint sets.
   * @return Pointer to the FootprintSetCollection object.
   */
  FootprintSetCollection::Ptr& footprintCollectionMgr()
  {
    return (*footprint_set_collection_ptr_);
  }

private:
  FootprintSetCollection::Ptr* footprint_set_collection_ptr_;
};
}  // namespace nav_core

#endif  // NAV_CORE_FOOTPRINT_SET_AWARE
