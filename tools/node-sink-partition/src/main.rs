// Standalone sink-hypergraph partitioner for the gsim node-sink planner.
//
// Input (file argument or stdin):
//   <vertices> <edges>
//   <vertex weights: `vertices` integers>
//   `edges` lines of: <edge weight> <num pins> <pin> ...   (0-based pins)
// Arguments: <k> [epsilon] (default epsilon 0.1)
// Output: `vertices` lines, one block id per vertex.
//
// Mirrors the p2sim repcut configuration: mt-kahypar deterministic preset,
// Objective::Cut, seed 42. Keeping this a subprocess avoids linking the
// simulator against the p2sim backend.

use std::io::{self, Read, Write};

fn fail(message: &str) -> ! {
    eprintln!("node-sink-partition: {}", message);
    std::process::exit(2);
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        fail("usage: node-sink-partition <input-file|-> <k> [epsilon]");
    }
    let path = args[1].clone();
    let k: i32 = args
        .get(2)
        .and_then(|value| value.parse().ok())
        .unwrap_or_else(|| fail("k must be an integer"));
    let epsilon: f64 = args
        .get(3)
        .and_then(|value| value.parse().ok())
        .unwrap_or(0.1);
    if k < 1 {
        fail("k must be positive");
    }

    let mut text = String::new();
    if path == "-" {
        io::stdin().read_to_string(&mut text).unwrap();
    } else {
        std::fs::File::open(&path)
            .unwrap_or_else(|error| fail(&format!("cannot open {}: {}", path, error)))
            .read_to_string(&mut text)
            .unwrap();
    }
    let mut tokens = text.split_ascii_whitespace();
    let mut next = || {
        tokens
            .next()
            .unwrap_or_else(|| fail("truncated hypergraph input"))
    };
    let vertices: usize = next().parse().unwrap();
    let edges: usize = next().parse().unwrap();
    let mut vertex_weights = Vec::with_capacity(vertices);
    for _ in 0..vertices {
        vertex_weights.push(next().parse::<i32>().unwrap());
    }
    let mut hyperedge_indices = Vec::with_capacity(edges + 1);
    let mut hyperedges = Vec::new();
    let mut hyperedge_weights = Vec::with_capacity(edges);
    hyperedge_indices.push(0usize);
    for _ in 0..edges {
        hyperedge_weights.push(next().parse::<i32>().unwrap());
        let pins: usize = next().parse().unwrap();
        for _ in 0..pins {
            let pin: usize = next().parse().unwrap();
            if pin >= vertices {
                fail("hyperedge pin out of range");
            }
            hyperedges.push(pin);
        }
        hyperedge_indices.push(hyperedges.len());
    }

    let parts = if k == 1 {
        vec![0i32; vertices]
    } else {
        mt_kahypar::initialize(1, false);
        let ctx = mt_kahypar::Context::builder()
            .preset(mt_kahypar::Preset::Deterministic)
            .k(k)
            .epsilon(epsilon)
            .objective(mt_kahypar::Objective::Cut)
            .seed(42)
            .build()
            .unwrap();
        let hypergraph = mt_kahypar::Hypergraph::from_adjacency(
            &ctx,
            vertices,
            &hyperedge_indices,
            &hyperedges,
            Some(&hyperedge_weights),
            Some(&vertex_weights),
        )
        .unwrap();
        let partitioned = hypergraph.partition().unwrap();
        eprintln!(
            "node-sink-partition: k={} imbalance={:.4} cut={}",
            k,
            partitioned.imbalance(),
            partitioned.cut()
        );
        partitioned.extract_partition()
    };
    if parts.len() != vertices {
        fail("partition size mismatch");
    }
    let stdout = io::stdout();
    let mut out = stdout.lock();
    for part in parts {
        writeln!(out, "{}", part).unwrap();
    }
}
