/**
 * @file state_snapshot.h
 * @brief FSM이 쓰고 다른 태스크가 읽는 현재 상태. 큐가 아니라 seqlock.
 *
 * 쓰기는 task_fsm 하나뿐이고 읽기는 여럿이다. 뮤텍스를 쓰면 낮은 우선순위
 * 독자가 FSM을 막을 수 있어, 쓰는 쪽이 절대 막히지 않는 seqlock을 쓴다.
 *
 *   쓰기:  seq++ (홀수) → 내용 복사 → seq++ (짝수)
 *   읽기:  seq 읽고(짝수 확인) → 복사 → seq 다시 읽어 같으면 성공, 아니면 재시도
 */
#pragma once
#include "flight_types.h"

void snapshot_init(role_t role);

/* task_fsm 전용. 다른 태스크가 부르면 안 된다. */
void snapshot_publish(const state_snapshot_t *s);

/* 누구나 호출 가능. 성공하면 true. 최대 3회 재시도 후 false. */
bool snapshot_read(state_snapshot_t *out);

/* 자주 쓰는 단일 값 조회 (내부에서 snapshot_read를 쓴다) */
flight_state_t snapshot_state(void);
role_t         snapshot_role(void);
