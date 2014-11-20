#ifndef __REGRESSION_CHECKER_H__
#define __REGRESSION_CHECKER_H__

/* Standard headers */
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

/* Boost headers */
#include "boost/noncopyable.hpp"
#include "boost/test/unit_test.hpp"
#include "boost/filesystem.hpp"
#include "boost/serialization/access.hpp"
#include "boost/serialization/split_member.hpp"
#include "boost/archive/xml_iarchive.hpp"
#include "boost/archive/xml_oarchive.hpp"
#include "boost/serialization/vector.hpp"

/* Common headers */
#include "point_t.h"
#include "quaternion_t.h"

/* Physics headers */
#include "inertia_tensor.h"
#include "physics_object.h"
#include "collision_info.h"
#include "physics_engine.h"

using namespace raptor_physics;

const float result_tolerance = 0.0005;
const std::string test_data_location = "test_data/";

#define CREATE_REGRESSION_CHECKER(NAME)  regression_checker NAME( \
    boost::unit_test::framework::get<boost::unit_test::test_suite>(boost::unit_test::framework::current_test_case().p_parent_id).p_name, \
    boost::unit_test::framework::current_test_case().p_name); \
    BOOST_LOG_TRIVIAL(fatal) << "PERF - Scene: " << boost::unit_test::framework::current_test_case().p_name;


/* Class to hold all data erquired to check the state of an object at the end of a frame */
class object_data
{
    public :
        object_data() = default;

        object_data(const physics_object &po, const int id)
        : _i(new inertia_tensor(po.get_inertia_tenor())), f(po.get_force()), tor(po.get_torque()), v(po.get_velocity()), w(po.get_angular_velocity()),
          o(po.get_orientation()), id(id) {  };

        object_data(const object_data &o)
        : _i(new inertia_tensor(*(o._i))), f(o.f), tor(o.tor), v(o.v), w(o.w), o(o.o), id(o.id) {  };

        const object_data& check(const object_data &rhs) const
        {
            /* Check they are the same object */
            BOOST_ASSERT(id == rhs.id);

            /* Check the interia tensor */
            BOOST_ASSERT((_i->mass() == rhs._i->mass()) || (fabs(_i->mass() - rhs._i->mass()) < result_tolerance));
            BOOST_ASSERT(fabs(magnitude(_i->center_of_mass() - rhs._i->center_of_mass())) < result_tolerance);
            BOOST_ASSERT(std::equal(&_i->tensor()[0], &_i->tensor()[6], rhs._i->tensor(), [](const float a, const float b)
                {
                    return (a == b) || (fabs(a - b) < result_tolerance);
                }));

            /* Check the dynamics */
            BOOST_ASSERT(fabs(magnitude(f   - rhs.f  )) < result_tolerance);
            BOOST_ASSERT(fabs(magnitude(tor - rhs.tor)) < result_tolerance);
            BOOST_ASSERT(fabs(magnitude(v   - rhs.v  )) < result_tolerance);
            BOOST_ASSERT(fabs(magnitude(w   - rhs.w  )) < result_tolerance);

            /* Check the orientation (com covers the translation) */
            BOOST_ASSERT(fabs(magnitude(o   - rhs.o  )) < result_tolerance);

            return *this;
        }

    private :
        friend class boost::serialization::access;

        template<class Archive>
        void save(Archive & ar, const unsigned int version) const
        {
            /* Yukky, but boost serialization cant cope with infinity so convert to an unlikely value */
            const fp_t mass = (_i->mass() == std::numeric_limits<fp_t>::infinity()) ? std::numeric_limits<fp_t>::max() : _i->mass();
            const point_t com(_i->center_of_mass());
            const fp_t *const tensor = _i->tensor();

            ar << BOOST_SERIALIZATION_NVP(mass);
            ar << BOOST_SERIALIZATION_NVP(com);
            for (int i = 0; i < 6; ++i)
            {
                const fp_t tensor_element = (tensor[i] == std::numeric_limits<fp_t>::infinity()) ? std::numeric_limits<fp_t>::max() : tensor[i];
                ar << BOOST_SERIALIZATION_NVP(tensor_element);
            }

            ar << BOOST_SERIALIZATION_NVP(f);
            ar << BOOST_SERIALIZATION_NVP(tor);
            ar << BOOST_SERIALIZATION_NVP(v);
            ar << BOOST_SERIALIZATION_NVP(w);
            ar << BOOST_SERIALIZATION_NVP(o);
            ar << BOOST_SERIALIZATION_NVP(id);
        }

        template<class Archive>
        void load(Archive & ar, const unsigned int version)
        {
            fp_t mass;
            point_t com;
            fp_t *tensor = new fp_t[6];

            /* Yukky, but boost serialization cant cope with infinity so convert from an unlikely value */
            ar >> BOOST_SERIALIZATION_NVP(mass);
            mass = (mass == std::numeric_limits<fp_t>::max()) ? std::numeric_limits<fp_t>::infinity() : mass;
            ar >> BOOST_SERIALIZATION_NVP(com);
            for (int i = 0; i < 6; ++i)
            {
                fp_t tensor_element;
                ar >> BOOST_SERIALIZATION_NVP(tensor_element);
                tensor[i] = (tensor_element == std::numeric_limits<fp_t>::max()) ? std::numeric_limits<fp_t>::infinity() : tensor_element;
            }
            _i.reset(new inertia_tensor(tensor, com, mass));

            ar >> BOOST_SERIALIZATION_NVP(f);
            ar >> BOOST_SERIALIZATION_NVP(tor);
            ar >> BOOST_SERIALIZATION_NVP(v);
            ar >> BOOST_SERIALIZATION_NVP(w);
            ar >> BOOST_SERIALIZATION_NVP(o);
            ar >> BOOST_SERIALIZATION_NVP(id);
        }

        template<class Archive>
        void serialize(Archive & ar, const unsigned int file_version)
        {
            boost::serialization::split_member(ar, *this, file_version);
        }

        std::unique_ptr<const inertia_tensor>   _i;
        point_t                                 f;
        point_t                                 tor;
        point_t                                 v;
        point_t                                 w;
        quaternion_t                            o;
        int                                     id;
};


/* Class to hold all data required to check the next or partial check the last collision of each frame */
class collision_data
{
    public :
        collision_data() = default;

        collision_data(const collision_info &c, const int i, const int j)
        : noc(c.get_normal_of_collision()), poc(c.get_point_of_collision()), t(c.get_time()), type(c.get_type()),
          po_i(i), po_j(j) {  };

        const collision_data& check(const collision_data &rhs) const
        {
            /* Check this is the same collision */
            BOOST_ASSERT(po_i == rhs.po_i);
            BOOST_ASSERT(po_j == rhs.po_j);

            /* Check the collision info */
            BOOST_ASSERT(fabs(magnitude(noc - rhs.noc)) < result_tolerance);
            BOOST_ASSERT(fabs(magnitude(poc - rhs.poc)) < result_tolerance);
            BOOST_ASSERT(fabs(t - rhs.t) < result_tolerance);
            BOOST_ASSERT(type == rhs.type);

            return *this;
        }

    private :
        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & BOOST_SERIALIZATION_NVP(noc);
            ar & BOOST_SERIALIZATION_NVP(poc);
            ar & BOOST_SERIALIZATION_NVP(t);
            ar & BOOST_SERIALIZATION_NVP(type);
            ar & BOOST_SERIALIZATION_NVP(po_i);
            ar & BOOST_SERIALIZATION_NVP(po_j);
        }

        point_t         noc;
        point_t         poc;
        fp_t            t;
        collision_t     type;
        int             po_i;
        int             po_j;
};


/* Class to hold all data required to check a frame */
class frame_data
{
    public :
        /* Default CTOR to create object to deserialise expected data into */
        frame_data() = default;

        /* CTOR from physics engine for actual data */
        frame_data(const physics_engine &pe, const int frame)
        : frame(frame)
        {
            /* Build objects */
            const int nr_objects = pe.next_object_id();
            for (int i = 0; i < nr_objects; ++i)
            {
                auto *object = pe.get_object(i);
                if (object != nullptr)
                {
                    objects.emplace_back(*object, i);
                }
            }

            /* Build collisions */
            for (int i = 0; i < nr_objects; ++i)
            {
                for (int j = 0; j < nr_objects; ++j)
                {
                    auto *col = pe.get_collision(i, j);
                    if (col != nullptr)
                    {
                        collisions.emplace_back(*col, i, j);
                    }
                }
            }
        };

        const frame_data& check(const frame_data &rhs) const
        {
            /* There arent even the same frame, give up now */
            BOOST_ASSERT(frame == rhs.frame);

            /* Check all objects */
            for (int i = 0; i < static_cast<int>(objects.size()); ++i)
            {
                objects[i].check(rhs.objects[i]);
            }

            /* Check all collisions */
            for (int i = 0; i < static_cast<int>(collisions.size()); ++i)
            {
                collisions[i].check(rhs.collisions[i]);
            }

            return *this;
        }

    private :
        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            /* Serialise frame number */
            ar & BOOST_SERIALIZATION_NVP(frame);

            /* Serialise objects */
            ar & BOOST_SERIALIZATION_NVP(objects);

            /* Serialise collisions */
            ar & BOOST_SERIALIZATION_NVP(collisions);
        }

        int                         frame;
        std::vector<object_data>    objects;
        std::vector<collision_data> collisions;
};


/* Top level class to check the output of a regression test */
class regression_checker : private boost::noncopyable
{
    public :
        regression_checker(const std::string &suite, const std::string& test)
        : _expected(), _actual(test_data_location + suite + "/" +  test + ".act"), _expected_archive(nullptr), _actual_archive(new boost::archive::xml_oarchive(_actual))
        {
            const std::string expected(test_data_location + suite + "/" + test + ".exp");
            if (boost::filesystem::exists(expected))
            {
                _expected.open(expected);
                _expected_archive = new boost::archive::xml_iarchive(_expected);
            }
        }
        ~regression_checker()
        {
            if (_expected.is_open())
            {
                delete _expected_archive;
                _expected.close();
            }

            delete _actual_archive;
            _actual.close();
        }

        regression_checker& check(const physics_engine &pe, const int frame)
        {
            /* Collect frame data */
            frame_data actual(pe, frame);
            
            /* Dump out actual, just in case */
            // const unsigned int flags = boost::archive::no_header;
            (*_actual_archive) << boost::serialization::make_nvp("frame_data", actual);
            _actual.flush();

            /* Check */
            if (_expected.is_open())
            {
                frame_data expected;
                (*_expected_archive) >> boost::serialization::make_nvp("frame_data", expected);
                expected.check(actual);
            }

            /* Every thing was fine */
            return *this;
        }

    private :
        std::ifstream                   _expected;
        std::ofstream                   _actual;
        boost::archive::xml_iarchive *  _expected_archive;
        boost::archive::xml_oarchive *  _actual_archive;
};

BOOST_CLASS_IMPLEMENTATION(point_t, object_serializable);
BOOST_CLASS_IMPLEMENTATION(quaternion_t, object_serializable);
BOOST_CLASS_IMPLEMENTATION(object_data, object_serializable);
BOOST_CLASS_IMPLEMENTATION(collision_data, object_serializable);
BOOST_CLASS_IMPLEMENTATION(frame_data, object_serializable);
BOOST_CLASS_IMPLEMENTATION(std::vector<object_data>, object_serializable);
BOOST_CLASS_IMPLEMENTATION(std::vector<collision_data>, object_serializable);

#endif /* #ifndef __REGRESSION_CHECKER_H__ */
