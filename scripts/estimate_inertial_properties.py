#!/usr/bin/env python3

from os import path, listdir
from pcg_gazebo.parsers.sdf import SDF, create_sdf_element
import sys
import trimesh
# Note: Both `trimesh` and `pcg-gazebo` can be installed via `pip`


def main():

    # Total mass taken from datasheet (given as ~18.0kg)
    total_mass = 18.0
    if len(sys.argv) > 1:
        # You can also use your own estimate of total mass if you managed to weigh Panda yourself :)
        total_mass = float(sys.argv[1])
    print('Estimating inertial properties for each link to add up to %f kg' % total_mass)

    # Get path to all visual meshes
    visual_mesh_dir = path.join(path.dirname(path.dirname(
        path.realpath(__file__))), 'panda', 'meshes', 'visual')
    visual_mesh_basenames = listdir(visual_mesh_dir)
    visual_mesh_basenames.sort()

    # Load all meshes
    meshes = {}
    for mesh_basename in visual_mesh_basenames:
        link_name = path.splitext(mesh_basename)[0]
        mesh_path = path.join(visual_mesh_dir, mesh_basename)
        meshes[link_name] = trimesh.load(mesh_path,
                                         force='mesh',
                                         ignore_materials=True)

    # Compute the total volume of the robot in order to estimate the required density
    total_volume = 0.0
    for link_name in meshes:
        mesh = meshes[link_name]
        print('Volume estimate of %s: %f m^3' % (link_name, mesh.volume))
        if link_name == 'finger':
            total_volume += 2*mesh.volume
            print('Note: Finger volume added twice to the total volume')
        else:
            total_volume += mesh.volume

    # Compute average density
    average_density = total_mass/total_volume
    print('Average density estimate: %f kg/m^3' % average_density)

    # Estimate inertial properties for each link
    mass = {}
    inertia = {}
    centre_of_mass = {}
    for link_name in meshes:
        mesh = meshes[link_name]
        mesh.density = average_density
        mass[link_name] = mesh.mass
        inertia[link_name] = mesh.moment_inertia
        centre_of_mass[link_name] = mesh.center_mass

    # Redistribute 75% of the hand's mass to the fingers due to internal mechanical coupling
    # This improves reliability of grasps and makes fingers less susceptible to disturbances
    pct_mass_of_hand = 0.75
    # Add half of hand's redistributed mass to each finger
    # Then, update inertial proportionally to mass increase ratio
    old_mass_finger = mass['finger']
    mass['finger'] += (pct_mass_of_hand/2)*mass['hand']
    inertia['finger'] *= mass['finger']/old_mass_finger
    # Recompute centre of finger's mass to account for this redistribution
    translation_hand_finger = [0.0, 0.0, 0.0584]
    centre_of_mass['finger'] = [
        old_mass_finger * centre_of_mass['finger'][0] +
        (pct_mass_of_hand/2) *
        mass['hand'] * (centre_of_mass['hand'][0]-translation_hand_finger[0]),
        old_mass_finger * centre_of_mass['finger'][1] +
        (pct_mass_of_hand/2) *
        mass['hand'] * (centre_of_mass['hand'][1]-translation_hand_finger[1]),
        old_mass_finger * centre_of_mass['finger'][2] +
        (pct_mass_of_hand/2) *
        mass['hand'] * (centre_of_mass['hand'][2]-translation_hand_finger[2]),
    ] / mass['finger']
    # Reduce mass and inertia of hand to 25%
    mass['hand'] *= (1.0-pct_mass_of_hand)
    inertia['hand'] *= (1.0-pct_mass_of_hand)

    # Create a new SDF with one model
    sdf = SDF()
    sdf.add_model(name='panda')
    model = sdf.models[0]

    # Set inertial properties for each link into the SDF
    for link_name in meshes:
        link = create_sdf_element('link')
        link.mass = mass[link_name]
        link.inertia.ixx = inertia[link_name][0][0]
        link.inertia.iyy = inertia[link_name][1][1]
        link.inertia.izz = inertia[link_name][2][2]
        link.inertia.ixy = inertia[link_name][0][1]
        link.inertia.ixz = inertia[link_name][0][2]
        link.inertia.iyz = inertia[link_name][1][2]
        link.inertial.pose = [centre_of_mass[link_name][0],
                              centre_of_mass[link_name][1],
                              centre_of_mass[link_name][2],
                              0.0, 0.0, 0.0]
        model.add_link(link_name, link)

    # Write into output file
    output_file = 'panda_inertial_out.sdf'
    sdf.export_xml(output_file)
    print('Results written into "%s"' % output_file)


if __name__ == "__main__":
    main()
