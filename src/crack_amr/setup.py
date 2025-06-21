from setuptools import find_packages, setup

package_name = 'crack_amr'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='heewoo',
    maintainer_email='wkrldowk1@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_to_person=crack_amr.robot7_to_person:main', #waypoint 4~7 한다음에 이동
            'crack_track=crack_amr.crack_tracking_inverse:main',
            
            'robot_go_to_pose=crack_amr.robot7_go_to_pose:main', #4~7을 gotopose를 이용해서 하는 방식으로 고친거
            'just_go_to_car=crack_amr.just_go_to_car:main', #아침에 이거 먼저 확인해보기 -gotopose로 차 있는 곳 가기만 있는 테스트 코드
            'way_pub=crack_amr.waypointpublish:main',
            'robot6=crack_amr.robot6_through:main', #waypoint원본 파일: 이거랑 구독 방식중에 뭘로 할지 결정
            'way_sub=crack_amr.waypointsubscriber:main', #웨이포인트를 구독하는 방식으로 하는 거-> 이거 되면 4~7 어떻게 할지 결정할듯

            'robot7_go_to_pose=crack_amr.robot7_go_to_pose:main',
            'robot7_through=crack_amr.robot7_through:main',
        ],
    },
)
