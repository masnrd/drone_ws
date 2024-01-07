"""
pathfinder:
Pathfinding algorithm for the drone
"""

from typing import Tuple
from .maplib import LatLon

DEFAULT_RESOLUTION = 0

HARDCODED_WAYPOINT_TUPLES = [
    (1.340643554050367, 103.9626564184675),(1.3406460120797241, 103.96266590490286), (1.3406553947973538, 103.96266868103588),(1.340662319486202, 103.9626619707333),(1.3406598614569591, 103.96265248429731),(1.3406504787387523, 103.96264970816453),(1.3406410960208917, 103.96264693203186),(1.340634171332953, 103.9626536423348),(1.3406366293624359, 103.96266312876999),(1.3406390873917953, 103.9626726152049),(1.3406484701089723, 103.96267539133794),(1.3406578528264808, 103.96267816747113),(1.340664777515328, 103.962671457169),(1.3406717022046244, 103.96266474686638),(1.3406692441755008, 103.96265526043022),(1.3406667861462604, 103.9626457739938),(1.340657403427601, 103.96264299786105),(1.3406480207092843, 103.96264022172844),(1.3406386379912962, 103.96263744559593),(1.3406317133033645, 103.96264415589933),(1.3406247886158882, 103.9626508662022),(1.3406272466454838, 103.96266035263724),(1.3406297046749613, 103.96266983907196),(1.3406321627043234, 103.96267932550641),(1.3406415454210412, 103.96268210163952),(1.3406509281380923, 103.96268487777273),(1.3406603108554942, 103.96268765390607),(1.3406672355443356, 103.96268094360441),(1.3406741602336387, 103.96267423330225),(1.3406810849233974, 103.9626675229996),(1.340678626894387, 103.96265803656327),(1.340676168865263, 103.96264855012666),(1.3406737108360192, 103.96263906368978),(1.340664328116905, 103.96263628755706),(1.3406549453981276, 103.96263351142449),(1.3406455626796867, 103.96263073529204),(1.340636179961592, 103.9626279591597),(1.340629255273653, 103.96263466946355),(1.3406223305861802, 103.9626413797669),(1.3406154058991617, 103.96264809006975),(1.3406178639288722, 103.96265757650458),(1.340620321958469, 103.96266706293916),(1.340622779987944, 103.96267654937346),(1.3406252380173105, 103.96268603580744),(1.340634620733567, 103.9626888119406),(1.34064400345017, 103.96269158807385),(1.3406533861671082, 103.96269436420724),(1.3406627688843908, 103.96269714034074),(1.3406696935732296, 103.96269043003954),(1.3406766182625336, 103.96268371973783),(1.3406835429522879, 103.96267700943564),(1.3406904676425027, 103.96267029913292),(1.3406880096136118, 103.96266081269643),(1.3406855515846088, 103.96265132625966),(1.3406830935554792, 103.9626418398226),(1.3406806355262408, 103.96263235338525),(1.3406712528066644, 103.9626295772526),(1.3406618700874295, 103.96262680112005),(1.3406524873685357, 103.96262402498765),(1.3406431046499818, 103.96262124885536),(1.340633721931771, 103.9626184727232),(1.3406267972438324, 103.9626251830275),(1.340619872556359, 103.96263189333129),(1.3406129478693358, 103.96263860363457),(1.3406060231827763, 103.96264531393739),(1.3406084812126031, 103.96265480037208),(1.3406109392423113, 103.96266428680649),(1.3406133972719092, 103.9626737732406),(1.340615855301387, 103.96268325967444),(1.3406183133307479, 103.96269274610798),(1.3406276960465584, 103.96269552224116),(1.3406370787627007, 103.96269829837446),(1.3406464614791809, 103.9627010745079),(1.3406558441960041, 103.96270385064146),(1.3406652269131705, 103.96270662677513),(1.34067215160201, 103.96269991647438),(1.340679076291308, 103.96269320617313),(1.340686000981068, 103.96268649587137),(1.3406929256712783, 103.96267978556912),(1.3406998503619538, 103.96267307526637),(1.3406973923331775, 103.96266358882971),(1.3406949343042875, 103.96265410239278),(1.3406924762752777, 103.96264461595554),(1.3406900182461603, 103.96263512951803),(1.3406875602169184, 103.96262564308023),(1.340678177496889, 103.96262286694761),(1.3406687947771931, 103.96262009081512),(1.340659412057845, 103.96261731468276),(1.3406500293388302, 103.96261453855051),(1.3406406466201586, 103.9626117624184),(1.3406312639018265, 103.9626089862864),(1.3406243392138901, 103.96261569659114),(1.340617414526414, 103.9626224068954),(1.340610489839395, 103.96262911719916),(1.3406035651528314, 103.96263582750241),(1.3405966404667229, 103.96264253780517),(1.340599098496674, 103.96265202423969),(1.3406015565264986, 103.96266151067393),(1.3406040145562095, 103.96267099710788),(1.3406064725858067, 103.96268048354155),(1.3406089306152884, 103.96268996997493),(1.3406113886446536, 103.962699456408),(1.34062077136, 103.96270223254123),(1.3406301540756878, 103.96270500867459),(1.3406395367917137, 103.96270778480806),(1.3406489195080826, 103.96271056094166),(1.3406583022247849, 103.96271333707539),(1.3406676849418382, 103.96271611320924),(1.3406746096306716, 103.96270940290894),(1.3406815343199703, 103.96270269260815),(1.340688459009726, 103.96269598230684),(1.3406953836999436, 103.96268927200504),(1.3407023083906118, 103.96268256170274),(1.3407092330817418, 103.96267585139995),(1.3407067750530832, 103.96266636496311),(1.3407043170243063, 103.962656878526),(1.3407018589954158, 103.96264739208861),(1.3406994009664115, 103.96263790565092),(1.3406969429372906, 103.96262841921296),(1.3406944849080544, 103.9626189327747),(1.3406851021875672, 103.96261615664214),(1.340675719467414, 103.96261338050971),(1.3406663367476033, 103.96261060437737),(1.3406569540281374, 103.96260782824518),(1.3406475713090096, 103.9626050521131),(1.3406381885902232, 103.96260227598115),(1.340628805871775, 103.96259949984933),(1.340621881183836, 103.96260621015452),(1.3406149564963572, 103.96261292045924),(1.3406080318093339, 103.96261963076344),(1.3406011071227695, 103.96262634106715),(1.3405941824366632, 103.96263305137036),(1.340587257751019, 103.96263976167306),(1.3405897157810767, 103.96264924810743),(1.340592173811022, 103.96265873454149),(1.3405946318408508, 103.96266822097527),(1.340597089870566, 103.96267770740876),(1.3405995479001642, 103.96268719384199),(1.340602005929647, 103.9626966802749),(1.3406044639590116, 103.96270616670753),(1.3406138466739035, 103.9627089428408),(1.340623229389129, 103.9627117189742),(1.3406326121047019, 103.96271449510773),(1.34064199482061, 103.96271727124137),(1.3406513775368611, 103.96272004737514),(1.3406607602534488, 103.96272282350904),(1.3406701429703778, 103.96272559964304),(1.3406770676592217, 103.9627188893432),(1.3406839923485194, 103.96271217904285),(1.3406909170382677, 103.96270546874202),(1.340697841728481, 103.96269875844067),(1.3407047664191527, 103.96269204813882),(1.34071169111028, 103.96268533783649),(1.3407186158018598, 103.96267862753365),(1.340716157773327, 103.96266914109665),(1.340713699744668, 103.96265965465938),(1.3407112417158968, 103.9626501682218),(1.3407087836870073, 103.96264068178395),(1.340706325658001, 103.96263119534582),(1.340703867628884, 103.96262170890739),(1.340701409599644, 103.96261222246869),(1.3406920268786964, 103.96260944633616),(1.3406826441580932, 103.96260667020375),(1.3406732614378252, 103.96260389407149),(1.340663878717903, 103.96260111793933),(1.3406544959983175, 103.96259834180731),(1.3406451132790704, 103.96259556567539),(1.3406357305601662, 103.96259278954362),(1.3406263478415954, 103.96259001341195),(1.3406194231536666, 103.96259672371762),(1.3406124984661836, 103.96260343402278),(1.3406055737791593, 103.96261014432744),(1.3405986490925939, 103.96261685463159),(1.3405917244064898, 103.96262356493526),(1.3405847997208413, 103.96263027523842),(1.3405778750356547, 103.96263698554108),(1.3405803330658286, 103.96264647197528),(1.3405827910958887, 103.96265595840917),(1.3405852491258352, 103.9626654448428),(1.3405877071556649, 103.96267493127613),(1.340590165185381, 103.96268441770917),(1.340592623214977, 103.96269390414191),(1.3405950812444642, 103.96270339057438),(1.3405975392738314, 103.96271287700657),(1.3406069219882657, 103.96271565313988),(1.3406163047030348, 103.96271842927332),(1.3406256874181488, 103.9627212054069),(1.3406350701336025, 103.96272398154059),(1.3406444528493928, 103.96272675767439),(1.3406538355655244, 103.96272953380833),(1.3406632182820022, 103.9627323099424),(1.3406726009988132, 103.96273508607656),(1.340679525687653, 103.96272837577717),(1.340686450376945, 103.96272166547729),(1.3406933750667016, 103.96271495517689),(1.340700299756906, 103.96270824487601),(1.3407072244475802, 103.96270153457462),(1.3407141491387065, 103.96269482427273),(1.34072107383029, 103.96268811397034),(1.3407279985223337, 103.96268140366746),
]

class DummyPathfinder:
    def __init__(self, res: int, center: Tuple):
        """
        `res`: H3 Resolution
        `center`: Starting position as tuple of lat, lon
        """
        self.res = res
        self.center_hex = None

    def find_next_step(self, current_position: Tuple[float, float], prob_map) -> Tuple[float, float]:
        #TODO: actual implementation uses H3 hexes as current_position and return, this one uses the actual latlon
        
        # Identify closest point in tuples
        cur_latlon = LatLon(current_position[0], current_position[1])
        distances = [cur_latlon.distFromPoint(LatLon(p[0], p[1])) for p in HARDCODED_WAYPOINT_TUPLES]
        closest_point_i = distances.index(min(distances))
        return HARDCODED_WAYPOINT_TUPLES[(closest_point_i + 1) % len(distances)]

class PathfinderState:
    def __init__(self, start_pos: LatLon, prob_map):
        start_tup = (start_pos.lat, start_pos.lon)
        self._pathfinder = DummyPathfinder(DEFAULT_RESOLUTION, start_tup)
        self._prob_map = prob_map

    def get_next_waypoint(self, cur_pos: LatLon) -> LatLon:
        #TODO: necessary conversions from latlon to h3 hexes
        cur_tup = (cur_pos.lat, cur_pos.lon)
        next_tup = self._pathfinder.find_next_step(cur_tup, self._prob_map)
        return LatLon(next_tup[0], next_tup[1])
    
    def found_signals(self, cur_pos: LatLon, signal_count: int):
        pass