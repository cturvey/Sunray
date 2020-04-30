/*
Ardumower Sunray App 
Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
Licensed GPLv3 for open source use
or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)
*/

function distance (a, b) {
  return Math.sqrt(squareDistance(a, b))
}

function squareDistance (a, b) {
  return (a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y)
}

function shortestPath (polys, graph, srcNode, dstNode) {
  var q = [{ X: srcNode.X, Y: srcNode.Y, path: [srcNode], currentLength: 0, heuristic: 0 }]
  var visited = {}

  while (q.length > 0) {
    //console.log('q',q);
    q.sort((a, b) => a.heuristic - b.heuristic) // this should be a queue
    const el = q.shift()
    //console.log('el',el);

    if (directPath(polys, dstNode, el)) {
      return [...el.path, dstNode]
    }
    visited[`${el.X} ${el.Y}`] = true

    const successors = graph[`${el.X} ${el.Y}`]
    //console.log('successors',successors);
    if (!successors) {
      continue
    }
    successors
      .filter(succ => !visited[`${succ.X} ${succ.Y}`])
      .forEach(succ => {
        const distToSucc = distance(succ, el)
        const newPath = el.path.slice()
        const currentLength = el.currentLength + distToSucc
        const heuristic = currentLength + distance(succ, dstNode)
        newPath.push({ X: succ.X, Y: succ.Y })
        q.push({ X: succ.X, Y: succ.Y, path: newPath, currentLength, heuristic })
      })
  }
}
