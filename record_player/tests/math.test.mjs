/** Run from record_player: npm test. Covers timestamp gaps, unwrapped lead, and signed gearing. */
import test from 'node:test';
import assert from 'node:assert/strict';
import {sampleIndex,dialLead,jointDialSpeed,degrees} from '../public/math.mjs';
import {frameMatrix} from '../public/robot.mjs';
import {Vector3} from 'three';

test('causal selection never borrows a future sample or holds through gaps',()=>{
  const times=[0,.1,.1,1]; // Includes duplicate timestamps and an explicit long gap.
  assert.equal(sampleIndex(times,-.01),-1);
  assert.equal(sampleIndex(times,.1),2);
  assert.equal(sampleIndex(times,.2),2);
  assert.equal(sampleIndex(times,.6),-1);
  assert.equal(sampleIndex(times,1),3);
  assert.equal(sampleIndex([],2),-1);
});
test('lead keeps sign and full turns instead of hiding winding or collision buildup',()=>{
  assert.equal(dialLead(370,Math.PI*2),10);
  assert.equal(dialLead(10,Math.PI*2),-350);
  assert.equal(dialLead(null,0),null);
  assert.equal(dialLead(0,undefined),null);
});
test('dial speed is mapped into signed joint units and preserves unknown values',()=>{
  assert.ok(Math.abs(jointDialSpeed(Math.PI,-.1)+18)<1e-10);
  assert.equal(jointDialSpeed(2,null),null);
  assert.equal(jointDialSpeed(null,-.1),null);
  assert.equal(degrees(NaN),null);
});
test('COMPAS frames preserve right-handed URDF transforms and metre translations',()=>{
  const transform=frameMatrix({point:[1,2,3],xaxis:[0,1,0],yaxis:[-1,0,0]}); // 90-degree Z rotation.
  assert.deepEqual(new Vector3(1,0,0).applyMatrix4(transform).toArray(),[1,3,3]);
  assert.deepEqual(new Vector3(0,0,1).applyMatrix4(transform).toArray(),[1,2,4]);
});
