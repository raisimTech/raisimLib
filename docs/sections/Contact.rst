#############################
Contact and Collision
#############################

Collision Group and Mask
=========================

Collision group and mask in RaiSim uses bit operations as most other physics engines. The below example is self-explanatory

.. code-block:: c

  raisim::World world;
  auto sphere0 = world.addSphere(1, 1, raisim::COLLISION(0), raisim::COLLISION(0) | raisim::COLLISION(1));
  auto sphere1 = world.addSphere(1, 1, raisim::COLLISION(1), raisim::COLLISION(0) | raisim::COLLISION(1));
  auto sphere2 = world.addSphere(1, 1, raisim::COLLISION(2), raisim::COLLISION(1));
  auto sphere3 = world.addSphere(1, 1, raisim::COLLISION(3), -1);

In the above example, ``sphere0`` is in collision group 0 and can collide with collision group 0 and 1.
``sphere1`` is in collision group 1 and can collide with collision group 0 and 1.
``sphere2`` is in collision group 2 and can collide with collision group 1.
``sphere3`` is in collision group 3 and can collide with any object (-1 means all groups).

**The collision group and mask is AND logic**.
In order for A and B collide, A should be able to collide with B and vice versa.

``sphere0`` can collide with ``sphere1``.
``sphere1`` cannot collide with ``spehre2`` (both conditions must be satisfied).
``sphere3`` cannot collide with any objects (because ``sphere0``, ``sphere1`` and ``sphere2`` cannot collide with the collision group 3).

By default, movable object are in collision group 1 and can collide with anything (collision mask = -1).
All static objects (e.g., ground and height map) are by default in collision group 63 and can collide with anything.

Contacts
=========================

:code:`raisim::Object` (and thus :code:`raisim::ArticulatedSystem` and :code:`raisim:SingleBodyObject`) have a method :code:`getContacts` which returns the list of contacts.
For example,

.. code-block:: c

  auto& contactsOnAnymal = anymal->getContacts();

Contact class is header-only and can be found at :code:`include/raisim/contact/Contact.hpp`.

Note that all vector quantities are expressed in the respective **Contact Frame**.
A contact frame is defined such that its z-axis is colinear with the contact normal and its origin is at the contact point. 
Its x- and y-axes are chosen arbitrary.
Here is an extensive example

.. code-block:: c

  /// Let's check all contact impulses acting on "LF_SHANK"
  auto footIndex = anymal->getBodyIdx("LF_SHANK");

  /// for all contacts on the robot, check ...
  for(auto& contact: anymal->getContacts()) {
    if (contact.skip()) continue; /// if the contact is internal, one contact point is set to 'skip'
    if ( footIndex == contact.getlocalBodyIndex() ) {
      std::cout<<"Contact impulse in the contact frame: "<<contact.getImpulse()->e()<<std::endl;
      /// the impulse is acting from objectB to objectA. You can check if this object is objectA or B by
      std::cout<<"is ObjectA: "<<contact.isObjectA()<<std::endl;
      std::cout<<"Contact frame: \n"<<contact.getContactFrame().e().transpose()<<std::endl;
      /// contact frame is transposed.
      std::cout<<"Contact impulse in the world frame: "<<contact.getContactFrame().e().transpose() * contact.getImpulse()->e()<<std::endl;
      std::cout<<"Contact Normal in the world frame: "<<contact.getNormal().e().transpose()<<std::endl;
      std::cout<<"Contact position in the world frame: "<<contact.getPosition().e().transpose()<<std::endl;
      std::cout<<"It collides with: "<<world.getObject(contact.getPairObjectIndex())<<std::endl;
      if (contact.getPairContactIndexInPairObject() != raisim::BodyType::STATIC) {
        /// Static objects do not have Contacts store. So you must check if the pair object is static
        /// This saves computation in raisim
        world.getObject(contact.getPairObjectIndex()).getContacts(); /// You can use the same methods on the pair object
      }
      std::cout<<"please check Contact.hpp for the full list of the methods"<<std::endl;
    }
  }

API
=========
You can get a vector of collisions on an object using ``raisim::Object::getContacts``.
Each element in the vector has the following API:

.. doxygenclass:: raisim::Contact
   :members:
