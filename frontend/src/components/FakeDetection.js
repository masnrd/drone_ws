function getDistanceFromLatLonInM(lat1, lon1, lat2, lon2) {
    var R = 6371;
    var dLat = deg2rad(lat2 - lat1);
    var dLon = deg2rad(lon2 - lon1);
    var a =
    Math.sin(dLat / 2) * Math.sin(dLat / 2) +
    Math.cos(deg2rad(lat1)) * Math.cos(deg2rad(lat2)) *
    Math.sin(dLon / 2) * Math.sin(dLon / 2)
    ;
    var c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    var d = R * c;
    return d * 1000;
}

function deg2rad(deg) {
    return deg * (Math.PI / 180)
}


function simpleHash(lat, lng) {
    const str = `${lat}:${lng}`;
    let hash = 0;
    for (let i = 0; i < str.length; i++) {
    const char = str.charCodeAt(i);
    hash = (hash << 5) - hash + char;
    hash = hash & hash;
    }
    return hash;
}

function shouldDisplayEntity(entity, clusters, threshold = 0.1) {
    const isWithinCluster = clusters.some(cluster =>
    getDistanceFromLatLonInM(entity.coordinates.lat, entity.coordinates.lon, cluster[0][0], cluster[0][1]) <= cluster[1]
    );

    if (isWithinCluster) return true;

    const hash = simpleHash(entity.coordinates.lat, entity.coordinates.lon);
    const pseudoRandom = Math.abs(hash % 100) / 100;
    return pseudoRandom < threshold;
}

export default {
    shouldDisplayEntity,
};