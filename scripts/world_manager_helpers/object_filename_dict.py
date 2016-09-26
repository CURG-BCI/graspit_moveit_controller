
import roslib
import roslib.packages
import rospy

#pkg_path = roslib.packages.get_pkg_dir('object_models') + '/'


def get_path(package_name, resource_name):
    resources = roslib.packages.find_resource(package_name, resource_name)
    if len(resources) == 0:
        rospy.logerr("Failed to find resource %s in package %s"%(resource_name, package_name))
        return ""
    else:
        return resources[0]


rock_file_name = get_path('object_models', 'darparock.iv')
drill_file_name = get_path('object_models', 'darpadrill.iv')
hammer_file_name = get_path('object_models', 'darpahammer.iv')
flashlight_file_name = get_path('object_models', 'darpaflashlight.iv')


large_shaving_gel_file_name = get_path('object_models', 'gillette_shaving_gel.ply')
coke_file_name = get_path('object_models', 'coke_can.iv')
odwalla_file_name = get_path('object_models', 'odwalla_bottle.iv')


milk_carton_file_name = get_path('object_models', 'milk_carton.vtk.ply')

#OLD STL MODELS
# all_file_name = get_path('object_models', 'all_convex.stl')
# garnier_file_name = get_path('object_models', 'garnier_shampoo_bottle_convex.stl')
# gillette_file_name = get_path('object_models', 'gillette_shaving_gel_convex.stl')

#NEW STL MODELS
all_file_name = get_path('object_models', 'all.vtk.stl')
garnier_file_name = get_path('object_models', 'garnier_shampoo_bottle.vtk.stl')
gillette_file_name = get_path('object_models', 'gillette_shaving_gel.vtk.stl')


drill_custom_file_name = get_path('object_models','drill_custom_in_meters.iv')
mug_custom_file_name = get_path('object_models','mug_custom_in_meters.iv')
darpaphonehandset_file_name = get_path('object_models','darpaphonehandset_1000_different_coordinate_system.iv')


box_file_name = get_path('object_models','box_in_meters.iv')
snapple_file_name = get_path('object_models','snapple_in_meters.iv')
library_cup_file_name = get_path('object_models','library_cup_in_meters.iv')
krylon_spray_file_name = get_path('object_models','krylon_spray_in_meters.iv')

frenchs_file_name = get_path('object_models', 'frenchs_classic_yellow_mustard_14oz_two.vtk.stl')
drill_two_file_name = get_path('object_models', 'drill_custom_two.vtk.stl')

file_name_dict = dict()

file_name_dict['garnier_shampoo_bottle'] = garnier_file_name
file_name_dict['all'] = all_file_name
file_name_dict['odwalla_bottle'] = odwalla_file_name
file_name_dict['darpaflashlight'] = flashlight_file_name
file_name_dict['gillette_shaving_gel'] = gillette_file_name
file_name_dict['milk_carton'] = milk_carton_file_name

file_name_dict['drill_custom'] = drill_custom_file_name
file_name_dict['mug_custom'] = mug_custom_file_name
file_name_dict['darpaphonehandset'] = darpaphonehandset_file_name
file_name_dict['box'] = box_file_name
file_name_dict['snapple'] = snapple_file_name
file_name_dict['darparock'] = rock_file_name
file_name_dict['krylon_spray'] = krylon_spray_file_name
file_name_dict['library_cup'] = library_cup_file_name

file_name_dict['frenchs_classic_yellow_mustard_14oz_two'] = frenchs_file_name
file_name_dict['drill_two'] = drill_two_file_name


file_name_dict['orange_wood_block_1inx1in'] = get_path('object_models', 'orange_wood_block_1inx1in.iv')
file_name_dict['black_and_decker_lithium_drill_driver.xml'] = get_path('object_models', 'black_and_decker_lithium_drill_driver.xml')
file_name_dict['melissa_doug_farm_fresh_fruit_banana'] = get_path('object_models', 'melissa_doug_farm_fresh_fruit_banana.iv')
file_name_dict['black_and_decker_lithium_drill_driver'] = get_path('object_models', 'black_and_decker_lithium_drill_driver.iv')
file_name_dict['cheerios_14oz'] = get_path('object_models', 'cheerios_14oz.iv')
file_name_dict['spam_12oz'] = get_path('object_models', 'spam_12oz.iv')
file_name_dict['pringles_original'] = get_path('object_models', 'pringles_original.iv')
file_name_dict['brine_mini_soccer_ball'] = get_path('object_models', 'brine_mini_soccer_ball.iv')
file_name_dict['red_metal_cup_white_speckles'] = get_path('object_models', 'red_metal_cup_white_speckles.iv')
file_name_dict['red_metal_plate_white_speckles'] = get_path('object_models', 'red_metal_plate_white_speckles.iv')
file_name_dict['play_go_rainbow_stakin_cups_1_yellow'] = get_path('object_models', 'play_go_rainbow_stakin_cups_1_yellow.iv')
file_name_dict['penn_raquet_ball'] = get_path('object_models', 'penn_raquet_ball.iv')
file_name_dict['play_go_rainbow_stakin_cups_2_orange'] = get_path('object_models', 'play_go_rainbow_stakin_cups_2_orange.iv')
file_name_dict['master_chef_ground_coffee_297g'] = get_path('object_models', 'master_chef_ground_coffee_297g.iv')
file_name_dict['melissa_doug_farm_fresh_fruit_orange'] = get_path('object_models', 'melissa_doug_farm_fresh_fruit_orange.iv')
file_name_dict['black_and_decker_lithium_drill_driver.vtk'] = get_path('object_models', 'black_and_decker_lithium_drill_driver.vtk')
file_name_dict['melissa_doug_farm_fresh_fruit_lemon'] = get_path('object_models', 'melissa_doug_farm_fresh_fruit_lemon.iv')
file_name_dict['clorox_disinfecting_wipes_35'] = get_path('object_models', 'clorox_disinfecting_wipes_35.iv')
# file_name_dict['frenchs_classic_yellow_mustard_14oz'] = get_path('object_models', 'frenchs_classic_yellow_mustard_14oz.iv')
file_name_dict['large_black_spring_clamp'] = get_path('object_models', 'large_black_spring_clamp.iv')
file_name_dict['melissa_doug_farm_fresh_fruit_strawberry'] = get_path('object_models', 'melissa_doug_farm_fresh_fruit_strawberry.iv')
file_name_dict['block_of_wood_6in'] = get_path('object_models', 'block_of_wood_6in.iv')
file_name_dict['champion_sports_official_softball'] = get_path('object_models', 'champion_sports_official_softball.iv')
file_name_dict['blue_wood_block_1inx1in'] = get_path('object_models', 'blue_wood_block_1inx1in.iv')
file_name_dict['domino_sugar_1lb'] = get_path('object_models', 'domino_sugar_1lb.iv')
file_name_dict['cheeze-it_388g'] = get_path('object_models', 'cheeze-it_388g.iv')
file_name_dict['melissa_doug_farm_fresh_fruit_pear'] = get_path('object_models', 'melissa_doug_farm_fresh_fruit_pear.iv')
file_name_dict['rubbermaid_ice_guard_pitcher_blue'] = get_path('object_models', 'rubbermaid_ice_guard_pitcher_blue.iv')
file_name_dict['purple_wood_block_1inx1in'] = get_path('object_models', 'purple_wood_block_1inx1in.iv')
file_name_dict['soft_scrub_2lb_4oz'] = get_path('object_models', 'soft_scrub_2lb_4oz.iv')
file_name_dict['play_go_rainbow_stakin_cups_5_green'] = get_path('object_models', 'play_go_rainbow_stakin_cups_5_green.iv')
file_name_dict['campbells_condensed_tomato_soup'] = get_path('object_models', 'campbells_condensed_tomato_soup.iv')
file_name_dict['melissa_doug_farm_fresh_fruit_apple'] = get_path('object_models', 'melissa_doug_farm_fresh_fruit_apple.iv')
file_name_dict['morton_salt_shaker'] = get_path('object_models', 'morton_salt_shaker.iv')
file_name_dict['jell-o_strawberry_gelatin_dessert'] = get_path('object_models', 'jell-o_strawberry_gelatin_dessert.iv')
file_name_dict['sponge_with_textured_cover'] = get_path('object_models', 'sponge_with_textured_cover.iv')
file_name_dict['play_go_rainbow_stakin_cups_3_red'] = get_path('object_models', 'play_go_rainbow_stakin_cups_3_red.iv')
file_name_dict['medium_black_spring_clamp'] = get_path('object_models', 'medium_black_spring_clamp.iv')
file_name_dict['comet_lemon_fresh_bleach'] = get_path('object_models', 'comet_lemon_fresh_bleach.iv')
file_name_dict['jell-o_chocolate_flavor_pudding'] = get_path('object_models', 'jell-o_chocolate_flavor_pudding.iv')
file_name_dict['block_of_wood_12in'] = get_path('object_models', 'block_of_wood_12in.iv')
file_name_dict['red_metal_bowl_white_speckles'] = get_path('object_models', 'red_metal_bowl_white_speckles.iv')
