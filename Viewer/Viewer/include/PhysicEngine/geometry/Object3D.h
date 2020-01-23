#pragma once


namespace PhysicEngine
{

	/**
	 * A generic 3D object 
	 *
	 * Object3D is an abstract class. This is the base class for all the 3D objects.
	 */
	class Object3D
	{
	public:

		/**
		 * Construct of Object3D
		 *
		 * Object3D is an abstract class. Every geometric 3D object used in the engine must derive from it.
		 */
		Object3D() {};

		/**
		 * Destructor
		 */
		virtual ~Object3D() {};
	};
}