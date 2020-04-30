/*
Ardumower Sunray App 
Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
Licensed GPLv3 for open source use
or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)
*/

function lineIntersects (p0, p1, p2, p3) {
  var s1x = p1.X - p0.X
  var s1y = p1.Y - p0.Y
  var s2x = p3.X - p2.X
  var s2y = p3.Y - p2.Y

  var s = (-s1y * (p0.X - p2.X) + s1x * (p0.Y - p2.Y)) / (-s2x * s1y + s1x * s2y)
  var t = (s2x * (p0.Y - p2.Y) - s2y * (p0.X - p2.X)) / (-s2x * s1y + s1x * s2y)

  return s > 0 && s < 1 && t > 0 && t < 1
}

function linePolygonIntersection (src, dst, poly) {
  //var c = polygon(o)
  var c = poly;
  for (var k = 0; k < c.length; k++) {
    for (var j = k+1; j < c.length; j++) {
      if (lineIntersects(c[k], c[(j) % c.length], src, dst)) {
        return true
      }    
    }
  }
  return false
}

function lineCircleIntersection (src, dst, o) {
  const dx = dst.X - src.X
  const dy = dst.Y - src.Y
  const dr2 = dx * dx + dy * dy
  const vect = (src.X - o.X) * (dst.Y - o.Y) - (dst.X - o.X) * (src.Y - o.Y)
  const discr = (o.radius + extraSpace) * (o.radius + extraSpace) * dr2 - vect * vect
  if (discr > 0) {
    var sy = dy > 0 ? 1 : -1
    var sqrtDiscr = Math.sqrt(discr)

    var collision1 = {
      x: (vect * dy - sy * dx * sqrtDiscr) / dr2 + o.X,
      y: (-vect * dx - Math.abs(dy) * sqrtDiscr) / dr2 + o.Y
    }
    var collision2 = {
      x: (vect * dy + sy * dx * sqrtDiscr) / dr2 + o.X,
      y: (-vect * dx + Math.abs(dy) * sqrtDiscr) / dr2 + o.Y
    }

    var mx = Math.min(src.X, dst.X)
    var mmx = Math.max(src.X, dst.X)
    var my = Math.min(src.Y, dst.Y)
    var mmy = Math.max(src.Y, dst.Y)

    if ((mx <= collision1.X && collision1.X <= mmx && my <= collision1.Y && collision1.Y <= mmy) ||
          (mx <= collision2.X && collision2.X <= mmx && my <= collision2.Y && collision2.Y <= mmy)) {
      return true
    }
  }
  return false
}

function directPath (polys, src, dst) {
  return polys
    .filter(o => src !== o)
    .every(o => !linePolygonIntersection(src, dst, o))
}

function buildGraph (polys, coords) {
  // Add edge from each vertex to all visible vertex
  /*const allVertices = coords
    .map(dst => polygon(dst))
    .reduce((a, b) => [...a, ...b])*/
  var allVertices = [];
  var polys = JSON.parse(JSON.stringify(polys));    
  polys.forEach(poly => {    
    poly.forEach(pt => {    
      allVertices.push(pt);
    });
  });  
  coords.forEach(pt => { 
    allVertices.push(pt);  
    polys.push([pt]);
  });
  //console.log('allVertices',allVertices);

  const graph = {}
  polys.forEach(src => {
    //const srcPoly = polygon(src)    
    const srcPoly = src
    //console.log('srcPoly',srcPoly);

    // Centers can also reach any visible vertices
    //srcPoly.push(src)

    srcPoly.forEach(srcP => {
      allVertices
        .filter(c => c.X !== srcP.X || c.Y !== srcP.Y)
        //.filter(c => squareDistance(c, srcP)>0)        
        .forEach(c => {
          if (directPath(polys, srcP, c)) {
            const key = `${srcP.X} ${srcP.Y}`
            if (graph[key] == null) {
              graph[key] = []
            }
            graph[key].push(c)
          }
        })
    })
  })
  return graph
}
