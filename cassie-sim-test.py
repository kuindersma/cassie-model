import os.path
from pydrake.all import (DiagramBuilder, FloatingBaseType, RigidBodyPlant,
                         RigidBodyTree, Simulator, VectorSystem, 
                         ConstantVectorSource, CompliantMaterial, 
                         CompliantContactModelParameters, DrakeVisualizer,
                         AddFlatTerrainToWorld)

from pydrake.multibody.rigid_body_tree import (FloatingBaseType, RigidBodyFrame,
    RigidBodyTree)

from pydrake.lcm import DrakeLcm
import numpy as np

rtree = RigidBodyTree("cassie/urdf/cassie.urdf",
                     FloatingBaseType.kRollPitchYaw)

plant = RigidBodyPlant(rtree)
builder = DiagramBuilder()
cassie = builder.AddSystem(plant)

# Setup contact stuff -- params lifted from PR2 C++ example
AddFlatTerrainToWorld(rtree)

kYoungsModulus = 1e7  # Pa
kDissipation = 100 # s/m
kStaticFriction = 1.0
kDynamicFriction = 5e-1

default_material = CompliantMaterial()
default_material.set_youngs_modulus(kYoungsModulus)
default_material.set_dissipation(kDissipation)
default_material.set_friction(kStaticFriction, kDynamicFriction)

cassie.set_default_compliant_material(default_material)

kStictionSlipTolerance = 1e-3 # m/s
kContactRadius = 2e-4 # m
params = CompliantContactModelParameters()
params.characteristic_radius = kContactRadius
params.v_stiction_tolerance = kStictionSlipTolerance
cassie.set_contact_model_parameters(params)


# Setup visualizer
lcm = DrakeLcm()
visualizer = builder.AddSystem(DrakeVisualizer(tree=rtree, 
        lcm=lcm, enable_playback=True))

builder.Connect(cassie.get_output_port(0), visualizer.get_input_port(0))


# Zero inputs -- passive forward simulation
u0 = ConstantVectorSource(np.zeros(rtree.get_num_actuators()))
null_controller = builder.AddSystem(u0)

builder.Connect(null_controller.get_output_port(0), cassie.get_input_port(0))

diagram = builder.Build()
simulator = Simulator(diagram)
simulator.set_target_realtime_rate(1.0)
simulator.set_publish_every_time_step(True)

state = simulator.get_mutable_context().get_mutable_continuous_state_vector()

# nominal standing state
state.SetFromVector([0., 0., 0.9342, 0., 0., 
                    0., 0., 0., 0.0057, 0.0057,
                    0.6726, 0.6726, -1.4100, -1.4100, -0.0374, 
                    -0.0374, 1.6493, 1.6493,-0.0289,-0.0289,
                    -1.7479,-1.7479, 0., 0., 0., 
                    0., 0., 0., 0., 0., 
                    0., 0., 0., 0., 0., 
                    0., 0., 0., 0., 0., 
                    0., 0., 0., 0.,])

simulator.StepTo(0.25)
