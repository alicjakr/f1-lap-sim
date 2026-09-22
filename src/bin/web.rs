use axum::extract::Query;
use axum::http::StatusCode;
use axum::response::{IntoResponse, Json};
use axum::routing::get;
use axum::Router;
use serde::Deserialize;
use tower_http::services::ServeDir;

use f1_lap_sim::api;

#[derive(Deserialize)]
struct SolveParams {
    track: String,
    #[serde(default = "default_spacing")]
    spacing: f64,
}

fn default_spacing() -> f64 {
    5.0 // see main.rs: 25 m reads 1-2 s optimistic, 5 m is within ~0.1 s of converged
}

async fn tracks() -> Json<Vec<api::TrackInfo>> {
    Json(api::available_tracks())
}

/// Below ~2 m the solve gets very large (and the geometry is resampled at 1 m anyway);
/// above ~100 m there are too few collocation points for a meaningful lap.
const MIN_SPACING_M: f64 = 2.0;
const MAX_SPACING_M: f64 = 100.0;

async fn solve(Query(params): Query<SolveParams>) -> impl IntoResponse {
    if !(MIN_SPACING_M..=MAX_SPACING_M).contains(&params.spacing) {
        return (
            StatusCode::BAD_REQUEST,
            format!("spacing must be between {MIN_SPACING_M} and {MAX_SPACING_M} m"),
        )
            .into_response();
    }
    // Ipopt's solve is synchronous/CPU-bound; must not block the async executor.
    let result = tokio::task::spawn_blocking(move || {
        api::solve_racing_line_for_track(&params.track, params.spacing)
    })
    .await
    .unwrap();

    match result {
        Ok(r) => (StatusCode::OK, Json(r)).into_response(),
        Err(e) => (StatusCode::BAD_REQUEST, e).into_response(),
    }
}

#[tokio::main]
async fn main() {
    let app = Router::new()
        .route("/api/tracks", get(tracks))
        .route("/api/solve", get(solve))
        .fallback_service(ServeDir::new("static"));

    let listener = tokio::net::TcpListener::bind("127.0.0.1:3000").await.unwrap();
    println!("Serving on http://127.0.0.1:3000");
    axum::serve(listener, app).await.unwrap();
}
