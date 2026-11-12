#!/usr/bin/env python3
"""Build a separate approximate linked-joint model from recorded joint feedback.

The upstream URDF and optimizer remain unchanged. The existing retargeter's
MimicJointKinematicAdaptor handles the four estimated distal couplings.
"""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def main():
    source = ROOT / 'upstream/dex5-pr321/assets/unitree_hand_Dex5/Dex5-URDF-L.urdf'
    fits = json.loads((ROOT / 'wuji_glove/config/distal-coupling-fit.json').read_text())
    tree = ET.parse(source)
    names = []
    ratios = {}
    for finger, fit in zip((2, 3, 4, 5), fits['fits']):
        follower, driver = f'Pitch_{finger}4L', f'Pitch_{finger}3L'
        joint = tree.find(f"joint[@name='{follower}']")
        if joint is None or joint.find('mimic') is not None:
            raise ValueError('Unexpected source hand model')
        ratio = fit['distal_ratio_zero_intercept']
        ET.SubElement(joint, 'mimic', joint=driver, multiplier=str(ratio), offset='0')
        names.append(follower)
        ratios[follower] = {'driver': driver, 'multiplier': ratio}
    # Keep the copied model's mesh references valid outside the upstream folder.
    for mesh in tree.findall('.//mesh'):
        mesh.set('filename', str((source.parent / mesh.get('filename')).resolve()))
    models = ROOT / 'wuji_glove/models'
    configs = ROOT / 'wuji_glove/config'
    models.mkdir(exist_ok=True)
    configs.mkdir(exist_ok=True)
    model = models / 'Dex5-left-linked.urdf'
    tree.write(model, encoding='utf-8', xml_declaration=True)
    active = ['Roll_21L', 'Pitch_22L', 'Pitch_23L',
              'Roll_31L', 'Pitch_32L', 'Pitch_33L',
              'Roll_41L', 'Pitch_42L', 'Pitch_43L',
              'Roll_51L', 'Pitch_52L', 'Pitch_53L',
              'Yaw_11L', 'Roll_12L', 'Pitch_13L', 'Pitch_14L']
    profile = {'retargeting': {'urdf_path': '../models/Dex5-left-linked.urdf',
                               'target_joint_names': active, 'scaling_factor': .9,
                               'low_pass_alpha': .8,
                               'project_dist': .015, 'escape_dist': .020},
               'provenance': {'basis': 'Approximate linear distal linkage fit to healthy H2 feedback',
                              'healthy_feedback_rows': fits['healthy_feedback_rows'],
                              'couplings': ratios,
                              'pinch_threshold_basis': 'Captured pinch gap 9.7-14.1 mm; claw gap 16.4-24.4 mm',
                              'physically_validated': False}}
    path = configs / 'left_dex5_retarget.json'
    path.write_text(json.dumps(profile, indent=2) + '\n')
    print(path)


if __name__ == '__main__':
    main()
