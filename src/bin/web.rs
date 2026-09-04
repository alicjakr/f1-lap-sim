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
    25.0
}

async fn tracks() -> Json<Vec<String>> {
    Json(api::available_tracks())
}

async fn solve(Query(params): Query<SolveParams>) -> impl IntoResponse {
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
